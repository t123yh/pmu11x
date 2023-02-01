//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_CONTROLLER_H
#define PMU11X_CTRL_CONTROLLER_H
#include <optional>
#include "battery.h"
#include "utils.h"
#include <FreeRTOS.h>
#include <task.h>

template <typename T> class AtomicStorage {
  std::optional<T> value;

public:
  void Set(bool ok, const T* dat) {
    CRITICAL_BLOCK;
    if (ok) {
      value = std::optional(*dat);
    } else {
      value = std::nullopt;
    }
  }

  std::optional<T> Get() {
    CRITICAL_BLOCK;
    return value;
  }
};

extern AtomicStorage<BatteryInfo> batteryInfoStorage;
extern AtomicStorage<RectifierInfo> rectifierInfoStorage;
void controllerTask(void* _);

#endif //PMU11X_CTRL_CONTROLLER_H
