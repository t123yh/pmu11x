//
// Created by 田韵豪 on 2023/1/23.
//

#ifndef PMU11X_CTRL_BATTERY_H
#define PMU11X_CTRL_BATTERY_H

#include <cstdint>
#include <cstddef>

const size_t kMaxCells = 16;
const size_t kMaxTempSensors = 4;
const size_t kMaxAlarms = 10;

struct BatteryAlarms {
  bool MosNtcError:1;
  bool GryoscopeError:1;
  bool GryoscopeLock:1;
  bool CurrentLimiterErrorAlert:1;
  bool VoltageLineBreakAlert:1;
  bool ChargeMosErrorAlert:1;
  bool DischargeMosErrorAlert:1;
  bool VoltageAdcErrorAlert:1;

  bool NtcLineBreakAlert:1;
  bool CurrentAdcErrorAlert:1;
  bool ChargeReverseProtection:1;
  bool CellLowVoltageProtection:1;
  bool :3;
  bool ComLockAlert:1;

  bool DischargeOverTempProtection:1;
  bool DischargeUnderTempProtection:1;
  bool PackOverVoltageProtection:1;
  bool StartErrorProtection:1;
  bool ChargeMosDisconnectStatus:1;
  bool DischargeMosDisconnectStatus:1;
  bool Comm536ErrorProtection:1;
  bool PackUnderVoltageProtection:1;

  bool ChargingStatus:1;
  bool DischargingStatus:1;
  bool ShortCircuitProtection:1;
  bool OverCurrentProtection:1;
  bool CellOverVoltageProtection:1;
  bool CellUnderVoltageProtection:1;
  bool ChargeOverTempProtection:1;
  bool ChargeUnderTempProtection:1;

  bool EnvUnderTempProtection:1;
  bool InvalidDryContactActionAlert:1;
  bool SeriousDryContactActionAlert:1;
  bool :2;
  bool CellVoltInvalidProtection:1;
  bool PackVoltInvalidAlert:1;
  bool CellVoltageDiffProtection:1;

  bool :4;
  bool HeatPadOnAlert:1;
  bool MosOverTempProtection:1;
  bool MosUnderTempProtection:1;
  bool StartChargeTempLowProtection:1;

  bool ChargeOverCurrentProtection:1;
  bool LimitingChargeCurrentStatus:1;
  bool DischargeOVerCurrentProtection:1;
  bool ExternalRelayBreakAlert:1;
  bool EnvironmentUnderTempProtection:1;
  bool EnvironmentOverTempProtection:1;
  bool :2;

  bool LowSoCAlert:1;
  bool :7;

  bool EnvironmentUnderTempAlert:1;
  bool EnvironmentOverTempAlert:1;
  bool PcbOverTempAlert:1;
  bool LowCapacityAlert:1;
  bool CellVoltageDiffAlert:1;
  bool MosOverTempAlert:1;

  bool CellOverVoltageAlert:1;
  bool CellUnderVoltageAlert:1;
  bool PackOverVoltageAlert:1;
  bool PackUnderVoltageAlert:1;
  bool ChargeOverCurrentAlert:1;
  bool DischargeOverCurrentAlert:1;
  bool CellChargeOverTempAlert:1;
  bool CellChargeUnderTempAlert:1;

  bool ChargeOverTempAlert:1;
  bool ChargeUnderTempAlert:1;
  bool DischargeOverTempAlert:1;
  bool DischargeUnderTempAlert:1;
};

struct BatteryInfo {
  uint8_t cellCount;

  struct __attribute__ ((__packed__)) {
    uint16_t value:13;
    bool underVoltage:1;
    bool overVoltage:1;
    bool balance:1;
  } cells[kMaxCells];
  int16_t current;
  uint16_t soc;
  uint16_t fullCapacity;

  uint8_t tempSensorCount;

  struct __attribute__ ((__packed__)) {
    uint8_t value;
    uint8_t status;
  } tempValues[kMaxTempSensors];

  uint8_t alarmCount;
  union {
    uint8_t alarms[kMaxAlarms];
    BatteryAlarms alarmInfo;
  };

  uint16_t loop;
  uint16_t sumCell;
  uint16_t soh;
};

void ControllerTask(void *_);

void BatteryInit();
bool GetBatteryInfo(BatteryInfo* info);

#endif //PMU11X_CTRL_BATTERY_H
