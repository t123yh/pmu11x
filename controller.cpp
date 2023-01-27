//
// Created by 田韵豪 on 2023/1/26.
//

#include "controller.h"
#include "battery.h"
#include "utils.h"
#include <FreeRTOS.h>
#include <task.h>
#include <optional>

static bool batteryCommOk;
static BatteryInfo batteryInfo;

void controllerTask (void*_) {
  while (1) {
    BatteryInfo localInfo;
    bool ok = GetBatteryInfo(&localInfo);
    {
      CRITICAL_BLOCK;
      batteryCommOk = ok;
      if (ok) {
        batteryInfo = localInfo;
      }
    }
    vTaskDelay(100);
  }
}

std::optional<BatteryInfo> getCurrentBatteryInfo() {
  CRITICAL_BLOCK;
  return batteryCommOk ? std::optional(batteryInfo) : std::nullopt;
}
