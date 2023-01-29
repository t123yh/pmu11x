//
// Created by 田韵豪 on 2023/1/28.
//

#ifndef PMU11X_CTRL_DS1302_H
#define PMU11X_CTRL_DS1302_H

#include <cstdint>
void ds1302Init();
void ds1302SetTime(const struct tm* t);
void ds1302GetTime(struct tm* t);
/*
void ds1302_read(uint8_t reg, uint8_t *dst, uint8_t read_len);
void ds1302_write_one(uint8_t reg, uint8_t dat);
void ds1302_write(uint8_t reg, uint8_t *src, uint8_t len);
 */
#endif //PMU11X_CTRL_DS1302_H
