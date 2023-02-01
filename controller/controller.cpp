//
// Created by 田韵豪 on 2023/1/26.
//

#include "controller.h"
#include "battery.h"
#include "rectifier.h"
#include "../rtt/SEGGER_RTT.h"
#include <utils.h>
#include <FreeRTOS.h>
#include <task.h>
#include <optional>

struct ControllerStatus {

};

bool ce = false;

bool ShouldCharge() {
  return ce;
}

AtomicStorage<BatteryInfo> batteryInfoStorage;
AtomicStorage<RectifierInfo> rectifierInfoStorage;
float ChargeCurrent0 = 3; // charge current when soc < 10%
float ChargeCurrent1 = 15; // charge current when soc between 10 and 90
float ChargeCurrent2 = 5; // charge current when soc > 90%

static const float ChargeCurrentAllowedDeviation = 0.3f;
static const float ChargeVoltageMaximumDelta = 0.7f;
static const float RectifierVoltageDefault = 50.0f;
float RectifierOnlineVoltage = 50;

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

void controllerTask(void *_) {
  uint32_t batteryFaultCycles = 0, rectifierFaultCycles = 0;
  while (1) {
    BatteryInfo localBattInfo;
    bool battCommOk = GetBatteryInfo(&localBattInfo);
    batteryInfoStorage.Set(battCommOk, &localBattInfo);
    if (!battCommOk) {
      batteryFaultCycles++;
    } else {
      batteryFaultCycles = 0;
    }

    RectifierInfo localRectInfo;
    bool rectOk = RectifierQuery(&localRectInfo);
    rectifierInfoStorage.Set(rectOk, &localRectInfo);

    if (!rectOk) {
      rectifierFaultCycles++;
      RectifierOnlineVoltage = 0;
    } else {
      if (rectifierFaultCycles != 0) {
        // Rectifier may have been re-plugged
        RectifierOfflineVoltageControl(RectifierVoltageDefault);
        SetRectifierVoltage(RectifierVoltageDefault);
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
      bool charge = ShouldCharge() && localBattInfo.soc < 99.9 && !battError;
      if (rectOk) {
        // TODO: when mains is not ok, alarmValue = 134349344. Should decide which bit actually means mains not ok
        bool mainsOk = !(localRectInfo.alarm_value & (1 << 27));
        bool rectifierOn = !(localRectInfo.alarm_value & (1 << 17));
        if (mainsOk) {
          // If mains is OK
          if (charge) {
            // If we want to start charging
            if (!rectifierOn) {
              SetRectifierVoltage(localBattInfo.voltage_sum + 0.4f);
              RectifierPowerControl(true);
            } else {
              if (battAlarms.ChargeMosDisconnectStatus) {
                // Rectifier voltage must be higher to start charging
                SetRectifierVoltage(localBattInfo.voltage_sum + 0.8f);
                SendBatteryCommand(BatteryCommandChargeEnable);
              } else {
                float batteryActualCurrent = -localBattInfo.current;
                float batteryTargetCurrent =
                    localBattInfo.soc < 10 ? ChargeCurrent0 :
                    localBattInfo.soc < 90 ? ChargeCurrent1 : ChargeCurrent2;
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
              RectifierPowerControl(true);
            } else if (battAlarms.DischargingStatus) {
              // Set rect voltage to slightly higher than batt to prevent discharging
              SetRectifierVoltage(localBattInfo.voltage_sum + 0.1f);
            }
          }
        } else {
          SetRectifierVoltage(localBattInfo.voltage_sum + 0.1f);
        }
      } else {
        // We don't want the battery to accidentally charge
        // in case the rectifier is connected again
        if (!battAlarms.ChargeMosDisconnectStatus) {
          SendBatteryCommand(BatteryCommandChargeDisable);
        }
      }
    } else {
      // If battery information is not available, put rectifier in a safe state
      SetRectifierVoltage(RectifierVoltageDefault);
    }

    vTaskDelay(100);
  }
}
