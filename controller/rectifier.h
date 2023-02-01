//
// Created by 田韵豪 on 2023/1/31.
//

#ifndef PMU11X_CTRL_RECTIFIER_H
#define PMU11X_CTRL_RECTIFIER_H

#include "rectifier.pb.h"

void RectifierInit();
bool RectifierPowerControl(bool on);

bool RectifierQuery(RectifierInfo* info);
extern bool rectifierActiveStatus;
extern bool rectifierRunningStatus;
bool RectifierOnlineVoltageControl(float voltage);
bool RectifierOfflineVoltageControl(float voltage);
bool RectifierPowerControl(bool on);

#endif //PMU11X_CTRL_RECTIFIER_H
