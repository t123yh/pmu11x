//
// Created by 田韵豪 on 2023/1/26.
//

#include "controller.h"
#include "battery.h"
#include "rectifier.h"
#include "../rtt/SEGGER_RTT.h"
#include "config.h"
#include <utils.h>
#include <FreeRTOS.h>
#include <task.h>
#include <optional>

AtomicStorage<BatteryInfo> batteryInfoStorage;
AtomicStorage<RectifierInfo> rectifierInfoStorage;

static const float ChargeCurrentAllowedDeviation = 0.3f;
static const float ChargeVoltageMaximumDelta = 0.7f;
float RectifierOnlineVoltage = 0;

bool SetRectifierVoltage(float val) {
  if (std::abs(val - RectifierOnlineVoltage) < 0.01)
    return true;
  bool ok = RectifierOnlineVoltageControl(RectifierOnlineVoltage);
  if (ok) {
    RectifierOnlineVoltage = val;
  }
  return ok;
}

void ResetVoltage() {
  RectifierOnlineVoltageControl(RectifierOnlineVoltage);
}

static BatteryInfo localBattInfo;
static RectifierInfo localRectInfo;
static PowerConfig localConfig;
static TickType_t rectifierOffLastSent;

static bool battCommOk;
static bool rectOk;

void sanitizeConfig() {
  if (localConfig.params.charge_current_90_100 > 30)
    localConfig.params.charge_current_90_100 = 30;
  if (localConfig.params.charge_current_5_90 > 30)
    localConfig.params.charge_current_5_90 = 30;
  if (localConfig.params.charge_current_0_5 > 30)
    localConfig.params.charge_current_0_5 = 30;
  if (localConfig.params.safe_voltage > 55)
    localConfig.params.safe_voltage = 55;
  if (localConfig.params.battery_low < 10)
    localConfig.params.battery_low = 10;
  if (localConfig.params.battery_critical < 3)
    localConfig.params.battery_critical = 3;
}

void controllerTask(void *_) {
  uint32_t batteryFaultCycles = 0, rectifierFaultCycles = 0;
  while (1) {
    localConfig = powerConfigItem.Get();
    sanitizeConfig();
    battCommOk = GetBatteryInfo(&localBattInfo);
    batteryInfoStorage.Set(battCommOk, &localBattInfo);
    if (!battCommOk) {
      batteryFaultCycles++;
    } else {
      batteryFaultCycles = 0;
    }

    rectOk = RectifierQuery(&localRectInfo);
    rectifierInfoStorage.Set(rectOk, &localRectInfo);

    if (!rectOk) {
      rectifierFaultCycles++;
      RectifierOnlineVoltage = 0;
    } else {
      if (rectifierFaultCycles != 0) {
        // Rectifier may have been re-plugged
        RectifierOfflineVoltageControl(localConfig.params.safe_voltage);
        SetRectifierVoltage(localConfig.params.safe_voltage);
      }
      rectifierFaultCycles = 0;
    }

    if (std::abs(localRectInfo.output_voltage - RectifierOnlineVoltage) > 0.2) {
      ResetVoltage();
    }

    if (battCommOk) {
      BatteryAlarms battAlarms;
      memcpy(&battAlarms, localBattInfo.alarm.bytes, sizeof(battAlarms));
      bool battError =
          battAlarms.ChargeMosErrorAlert || battAlarms.ChargeOverTempProtection || battAlarms.ChargeOverTempProtection
          || battAlarms.ChargeReverseProtection || battAlarms.ChargeUnderTempProtection ||
          battAlarms.ChargeUnderTempAlert
          || battAlarms.Comm536ErrorProtection || battAlarms.CurrentAdcErrorAlert || battAlarms.CurrentLimiterErrorAlert
          || battAlarms.EnvUnderTemp || battAlarms.EnvironmentOverTempAlert || battAlarms.EnvironmentOverTempProtection
          || battAlarms.EnvironmentUnderTempProtection || battAlarms.EnvironmentUnderTempAlert ||
          battAlarms.MosOverTempProtection
          || battAlarms.MosOverTempAlert || battAlarms.NtcLineBreakAlert || battAlarms.PackOverVoltageAlert
          || battAlarms.PackOverVoltageProtection || battAlarms.PackVoltInvalidAlert || battAlarms.PcbOverTempAlert
          || battAlarms.SeriousDryContactActionAlert || battAlarms.VoltageAdcErrorAlert ||
          battAlarms.VoltageLineBreakAlert;
      bool charge = !localConfig.charge_disable && localBattInfo.soc < 99.9 && !battError;
      if (rectOk) {
        // If mains supply is OK: bit 27
        // TODO: when mains is not ok, alarmValue = 134349344. Should investigate which bit actually means mains not ok
        bool mainsOk = !(localRectInfo.alarm_value & (1 << 27));
        // If rectifier is currently turned on: bit 17
        bool rectifierOn = !(localRectInfo.alarm_value & (1 << 17));
        // If battery SoC is enough, and mains supply is specified to be disabled
        // then rectifierShouldBeOff = true.
        // If battery SoC is low, do not turn off rectifier
        bool rectifierShouldBeOff =
            localConfig.params.battery_low < localBattInfo.soc && localConfig.mains_supply_disable;
        if (mainsOk && !rectifierShouldBeOff) {
          // If mains is OK, and rectifier should be powered on
          // then check whether we want to charge
          if (charge) {
            // If we want to start charging
            if (!rectifierOn) {
              // If rectifier is not turned on, then turn it on immediately
              SetRectifierVoltage(localBattInfo.voltage_sum + 0.3f);
              rectifierOffLastSent = 0;
              RectifierPowerControl(true);
            } else {
              if (battAlarms.ChargeMosDisconnectStatus) {
                // Rectifier voltage must be higher to start charging
                SetRectifierVoltage(localBattInfo.voltage_sum + 0.8f);
                SendBatteryCommand(BatteryCommandChargeEnable);
              } else {
                float batteryActualCurrent = -localBattInfo.current;
                float batteryTargetCurrent =
                    localBattInfo.soc < 5.0f ? localConfig.params.charge_current_0_5 :
                    localBattInfo.soc < 90.0f ? localConfig.params.charge_current_5_90
                                              : localConfig.params.charge_current_90_100;
                if (RectifierOnlineVoltage < localBattInfo.voltage_sum) {
                  RectifierOnlineVoltage = localBattInfo.voltage_sum;
                }
                if (batteryActualCurrent >= batteryTargetCurrent + ChargeCurrentAllowedDeviation ||
                    battAlarms.ChargeOverCurrentAlert ||
                    battAlarms.LimitingChargeCurrentStatus) {
                  SetRectifierVoltage(RectifierOnlineVoltage - 0.2f);
                } else if (batteryActualCurrent < batteryTargetCurrent - ChargeCurrentAllowedDeviation) {
                  // Limit voltage difference to 1V
                  if (RectifierOnlineVoltage < localBattInfo.voltage_sum + ChargeVoltageMaximumDelta) {
                    SetRectifierVoltage(RectifierOnlineVoltage + 0.05f);
                  }
                }
              }
            }
          } else {
            // We don't want to charge
            if (!battAlarms.ChargeMosDisconnectStatus) {
              // If charge mos is connected, disconnect it
              SendBatteryCommand(BatteryCommandChargeDisable);
            }
            // We want to discharge the battery neither
            if (!rectifierOn) {
              // Turn rectifier on
              SetRectifierVoltage(localBattInfo.voltage_sum + 0.1f);
              rectifierOffLastSent = 0;
              RectifierPowerControl(true);
            } else if (battAlarms.DischargingStatus) {
              // Set rect voltage to slightly higher than batt to prevent discharging
              SetRectifierVoltage(localBattInfo.voltage_sum + 0.1f);
            }
          }
        } else if (rectifierShouldBeOff) {
          // Rectifier should be off
          // r4850g2 need to receive off message periodically, or it will turn on
          if (rectifierOffLastSent == 0 || xTaskGetTickCount() - rectifierOffLastSent > 1000) {
            RectifierPowerControl(false);
            rectifierOffLastSent = xTaskGetTickCount();
          }
        } else if (!mainsOk) {
          // Rectifier should be on, but mains bad,
          // then set voltage slight higher than battery voltage
          // so if mains recovers, battery will stop discharging
          SetRectifierVoltage(localBattInfo.voltage_sum + 0.1f);
          // also disconnect battery charge mos
          // (it will be automatically connected back on if we need charge again)
          if (!battAlarms.ChargeMosDisconnectStatus) {
            SendBatteryCommand(BatteryCommandChargeDisable);
          }
        }
      } else {
        // Rectifier communication failed
        // We don't want the battery to accidentally charge
        // in case the rectifier is connected again,
        // so disconnect battery charge mos
        if (!battAlarms.ChargeMosDisconnectStatus) {
          SendBatteryCommand(BatteryCommandChargeDisable);
        }
      }
    } else {
      // If battery information is not available, put rectifier to a safe state
      SetRectifierVoltage(localConfig.params.safe_voltage);
    }

    vTaskDelay(100);
  }
}
