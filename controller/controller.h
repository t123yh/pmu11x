//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_CONTROLLER_H
#define PMU11X_CTRL_CONTROLLER_H
#include <optional>
#include "battery.h"
#include <utils.h>
#include <FreeRTOS.h>
#include <task.h>

extern AtomicStorage<BatteryInfo> batteryInfoStorage;
extern AtomicStorage<RectifierInfo> rectifierInfoStorage;

extern bool controllerChargeDisable, controllerMainsDisable;

void controllerTask(void* _);

#endif //PMU11X_CTRL_CONTROLLER_H
