//
// Created by 田韵豪 on 2023/2/2.
//

#ifndef PMU11X_CTRL_CONFIG_H
#define PMU11X_CTRL_CONFIG_H

#include <messages.pb.h>
#include <utils.h>
#include "../fs.h"
#include "../rtt/SEGGER_RTT.h"
#include <pb_encode.h>
#include <pb_decode.h>

pb_istream_t pb_istream_from_lfs_file(lfs_file* f);
pb_ostream_t pb_ostream_from_lfs_file(lfs_file* f);

template <typename T, const pb_msgdesc_t & desc> struct ConfigItem {
  AtomicStorage<T> value;
  const char* path;
  const T& default_value;

  ConfigItem(const char* file_path, const T& def) : path(file_path), default_value(def) {
  }

  T Get() {
    auto v = value.Get();
    if (v.has_value()) {
      return v.value();
    } else {
      return default_value;
    }
  }

  void Load() {
    lfs_file f;
    int open_ret = lfs_file_open(&lfs_instance, &f, path, LFS_O_RDONLY);
    if (open_ret < 0) {
      Update(&default_value);
      return;
    }
    auto stream = pb_istream_from_lfs_file(&f);
    T temp;
    bool ok = pb_decode(&stream, &desc, &temp);
    value.Set(ok, &temp);
    lfs_file_close(&lfs_instance, &f);
  }

  bool Update(const T* new_val) {
    lfs_file f;
    int ret = lfs_file_open(&lfs_instance, &f, path, LFS_O_WRONLY | LFS_O_CREAT);
    if (ret < 0) {
      return false;
    }
    auto stream = pb_ostream_from_lfs_file(&f);
    bool ok = pb_encode(&stream, &desc, new_val);
    ret = lfs_file_close(&lfs_instance, &f);
    if (ok && ret >= 0) {
      value.Set(true, new_val);
      return true;
    }
    return false;
  }
};

extern ConfigItem<NetworkConfig, NetworkConfig_msg> networkConfigItem;
extern ConfigItem<ChargeConfig, ChargeConfig_msg> chargeConfigItem;
void LoadConfigFromFilesystem();

#endif //PMU11X_CTRL_CONFIG_H
