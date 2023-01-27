//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_CONTROLLER_H
#define PMU11X_CTRL_CONTROLLER_H
#include <optional>
#include "battery.h"

void controllerTask(void* _);
std::optional<BatteryInfo> getCurrentBatteryInfo();

#endif //PMU11X_CTRL_CONTROLLER_H
