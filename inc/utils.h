//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_UTILS_H
#define PMU11X_CTRL_UTILS_H

#include <FreeRTOS.h>

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

#endif //PMU11X_CTRL_UTILS_H
