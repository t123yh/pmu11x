//
// Created by 田韵豪 on 2023/1/27.
//

#ifndef PMU11X_CTRL_CORE_SNTP_CONFIG_H
#define PMU11X_CTRL_CORE_SNTP_CONFIG_H

#include "../rtt/SEGGER_RTT.h"

static int my_printf(const char * sFormat, ...) {
  int r;
  va_list ParamList;

  va_start(ParamList, sFormat);
  r = SEGGER_RTT_vprintf(0, sFormat, &ParamList);
  va_end(ParamList);
  SEGGER_RTT_printf(0, "\n");
  return r;
}

#define LogError( message ) my_printf message
#define LogWarn( message ) my_printf message
#define LogInfo( message ) my_printf message

#endif //PMU11X_CTRL_CORE_SNTP_CONFIG_H
