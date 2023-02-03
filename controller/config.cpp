//
// Created by 田韵豪 on 2023/2/2.
//

#include "config.h"
#include <pb_decode.h>
#include <pb_encode.h>

AtomicStorage<NetworkConfig> currentNetworkConfig;
AtomicStorage<PowerConfig> currentPowerConfig;

static const NetworkConfig defaultNetworkConfig = {
    .ntp_server = {"ntp.aliyun.com", "ntp.tencent.com", "time.asia.apple.com"},
    .which_ip_config = NetworkConfig_dhcp_tag,
    .ip_config = {
        .dhcp = DhcpConfig_init_default
    }
};

static const PowerConfig defaultPowerConfig = PowerConfig_init_default;

ConfigItem<NetworkConfig, NetworkConfig_msg> networkConfigItem("/network_config.pb", defaultNetworkConfig);
ConfigItem<PowerConfig , PowerConfig_msg> powerConfigItem("/power_config.pb", defaultPowerConfig);

void LoadConfigFromFilesystem() {
  networkConfigItem.Load();
  powerConfigItem.Load();
}

static bool pb_lfs_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count) {
  auto *dest = (lfs_file *) stream->state;
  return lfs_file_write(&lfs_instance, dest, buf, count) == count;
}

pb_ostream_t pb_ostream_from_lfs_file(lfs_file *f) {
  pb_ostream_t stream;
  stream.callback = &pb_lfs_write;
  stream.state = f;
  stream.max_size = SIZE_MAX;
  stream.bytes_written = 0;
#ifndef PB_NO_ERRMSG
  stream.errmsg = NULL;
#endif
  return stream;
}

static bool pb_lfs_read(pb_istream_t *stream, pb_byte_t *buf, size_t count) {
  auto *dest = (lfs_file *) stream->state;
  return lfs_file_read(&lfs_instance, dest, buf, count) > 0;
}

pb_istream_t pb_istream_from_lfs_file(lfs_file *f) {
  pb_istream_t stream;
  stream.callback = &pb_lfs_read;
  stream.state = f;
  int size = lfs_file_seek(&lfs_instance, f, 0, LFS_SEEK_END);
  stream.bytes_left = size;
  lfs_file_seek(&lfs_instance, f, 0, LFS_SEEK_SET);
#ifndef PB_NO_ERRMSG
  stream.errmsg = NULL;
#endif
  return stream;
}