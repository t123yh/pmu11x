//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_UTILS_H
#define PMU11X_CTRL_UTILS_H

#include <FreeRTOS.h>
#include <optional>

struct CriticalAreaHandle
{
  inline __attribute__((always_inline)) CriticalAreaHandle() {
    portENTER_CRITICAL();
  }

  inline __attribute__((always_inline)) ~CriticalAreaHandle() {
    portEXIT_CRITICAL();
  }
};

#define CRITICAL_BLOCK CriticalAreaHandle __handle__;

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


#endif //PMU11X_CTRL_UTILS_H
