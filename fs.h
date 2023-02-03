//
// Created by 田韵豪 on 2023/1/30.
//

#ifndef PMU11X_CTRL_FS_H
#define PMU11X_CTRL_FS_H

#include "ws/mongoose.h"
#include <lfs.h>

void lfsInit();
extern struct mg_fs mg_fs_littlefs;
extern lfs_t lfs_instance;

#endif //PMU11X_CTRL_FS_H
