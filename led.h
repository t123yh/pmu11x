//
// Created by 田韵豪 on 2023/1/26.
//

#ifndef PMU11X_CTRL_LED_H
#define PMU11X_CTRL_LED_H

#include <FreeRTOS.h>
#include <timers.h>
#include <pico/types.h>

class Led {
public:
  enum LedMode {
    DISABLED,
    TRIGGER,
    REPEAT,
    ALWAYS_ON
  };
private:
  uint gpio;

  void On();
  void Off();
  void Period();

  TickType_t last;
  static void Callback(TimerHandle_t t);

public:
  bool IsOn() const;
  void Init();
  TickType_t period;
  explicit Led(uint gpio, TickType_t period) : gpio(gpio), last(0), period(period), mode(DISABLED) {}
  LedMode mode;
};

extern Led SysBlueLed;
extern Led SysYellowLed;
extern Led ComGreenLed;
extern Led ComYellowLed;

void InitLed();

#endif //PMU11X_CTRL_LED_H
