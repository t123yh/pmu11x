//
// Created by 田韵豪 on 2023/1/26.
//

#include <pico/types.h>
#include "led.h"
#include <FreeRTOS.h>
#include <timers.h>
#include <cstdio>
#include <hardware/gpio.h>

static void LedTimerFunction(TimerHandle_t t);

void Led::On() {
  gpio_put(gpio, false);
}

void Led::Off() {
  gpio_put(gpio, true);
}

bool Led::IsOn() const {
  return !gpio_get_out_level(gpio);
}

void Led::Init() {
  Off();
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
  auto tmr = xTimerCreate("LED", 7, pdTRUE, this, Callback);
  xTimerStart(tmr, portMAX_DELAY);
}

void Led::Period() {
  TickType_t current = xTaskGetTickCount();
  if (current > last + period) {
    if (mode == TRIGGER) {
      if (!IsOn()) {
        On();
      } else {
        Off();
        mode = DISABLED;
      }
      last = current;
    } else if (mode == REPEAT) {
      last = current;
      if (!IsOn()) {
        On();
      } else {
        Off();
      }
    } else if (mode == ALWAYS_ON) {
      On();
    } else {
      // Do not update last here, to turn led on as early as possible
      Off();
    }
  }
}

void Led::Callback(TimerHandle_t t) {
  static_cast<Led*>(pvTimerGetTimerID(t))->Period();
}

Led SysBlueLed(19, 200);
Led SysYellowLed(20, 200);
Led ComGreenLed(11, 40);
Led ComYellowLed(10, 40);

void InitLed() {
  SysBlueLed.Init();
  SysYellowLed.Init();
  ComGreenLed.Init();
  ComYellowLed.Init();
}