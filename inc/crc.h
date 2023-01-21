//
// Created by 田韵豪 on 2023/1/16.
//

#ifndef PMU11X_CTRL_CRC_H
#define PMU11X_CTRL_CRC_H


#ifdef __cplusplus
extern "C" {
#endif

unsigned int
xcrc32 (const unsigned char *buf, int len, unsigned int init);

#ifdef __cplusplus
}
#endif

#endif //PMU11X_CTRL_CRC_H
