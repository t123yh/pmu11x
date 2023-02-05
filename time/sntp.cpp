//
// Created by 田韵豪 on 2023/2/6.
//

#include <ctime>
#include "sntp.h"
#include "../ws/mongoose.h"
#include "ds1302.h"
#include "messages.pb.h"
#include "../controller/config.h"

static int retries;
static struct mg_connection *s_sntp_conn;
static void sfn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_SNTP_TIME) {
    int64_t ms = *(int64_t*)ev_data;
    ms /= 1000;
    tm tm{};
    ds1302SetTime(localtime_r(&ms, &tm));
    retries = 0;
  } else if (ev == MG_EV_CLOSE) {
    s_sntp_conn = nullptr;
  }
}

static int ntp_server_index;
static void SntpCallback(void *arg) {
  mg_mgr *mgr = (struct mg_mgr *) arg;
  if (!s_sntp_conn) {
    NetworkConfig netcfg = networkConfigItem.Get();
    ntp_server_index++;
    ntp_server_index %= netcfg.ntp_server_count;
    char server_addr[35];
    snprintf(server_addr, sizeof(server_addr), "udp://%s:123", netcfg.ntp_server[ntp_server_index]);
    s_sntp_conn = mg_sntp_connect(mgr, server_addr, sfn, NULL);
  }
  if (s_sntp_conn) {
    if (retries++ > 3) {
      mg_close_conn(s_sntp_conn);
      s_sntp_conn = nullptr;
    } else {
      mg_sntp_request(s_sntp_conn);
    }
  }
}

void AddSntpCallback(mg_mgr *mgr) {
  mg_timer_add(mgr, 500000, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, SntpCallback, mgr);
}
