//
// Created by 田韵豪 on 2023/2/5.
//

#include <ctime>
#include <pb_encode.h>
#include <hardware/adc.h>
#include "http.h"
#include "../time/ds1302.h"
#include "../fs/fs.h"
#include "controller.h"

static const char* kCorsHeader = "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST\r\n";
static const char* kWeekdayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const char* kMonthNames[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

void HttpHandler::MakeExtraHeaders() {
  tm time_info{};
  ds1302GetTime(&time_info);
  mg_snprintf(extra_headers_, sizeof(extra_headers_), "Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n%s",
              kWeekdayNames[time_info.tm_wday],
              time_info.tm_mday,
              kMonthNames[time_info.tm_mon],
              time_info.tm_year + 1900,
              time_info.tm_hour,
              time_info.tm_min,
              time_info.tm_sec,
              kCorsHeader);
}

static bool pb_mg_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count) {
  mg_connection *dest = (mg_connection *) stream->state;
  mg_send(dest, buf, count);
  return true;
}

static pb_ostream_t pb_ostream_from_mg(mg_connection *conn) {
  pb_ostream_t stream;
  stream.callback = &pb_mg_write;
  stream.state = conn;
  stream.max_size = SIZE_MAX;
  stream.bytes_written = 0;
#ifndef PB_NO_ERRMSG
  stream.errmsg = NULL;
#endif
  return stream;
}

void HttpHandler::ServeProtobuf(const pb_msgdesc_t *fields, const void *src_struct) {
  size_t size = 0;
  pb_get_encoded_size(&size, fields, src_struct);
  mg_printf(c_, "HTTP/1.1 200 OK\r\n"
               "Cache-Control: no-cache\r\n"
               "Content-Type: application/x-protobuf\r\n"
               "Connection: close\r\n"
               "Content-Length: %d\r\n"
               "%s"
               "\r\n", size, extra_headers_);
  pb_ostream_t stream = pb_ostream_from_mg(c_);
  pb_encode(&stream, fields, src_struct);
}

static char *rtrim(char *s)
{
  char* back = s + strlen(s);
  while(isspace(*--back));
  *(back+1) = '\0';
  return s;
}

void HttpHandler::HandleRequest(mg_connection *c, mg_http_message *msg) {
  c_ = c;
  hm_ = msg;

  MakeExtraHeaders();
  Authenticate();
  authenticated_ = true;
  if (mg_http_match_uri(hm_, "/query/battery")) {
    ServeQueryBattery();
  } else if (mg_http_match_uri(hm_, "/query/rectifier")) {
    ServeQueryRectifier();
  } else if (mg_http_match_uri(hm_, "/query/bus-voltage")) {
    ServeQueryAdc();
  } else if (mg_http_match_uri(hm_, "/sys/#")&& mg_vcmp(&hm_->method, "POST") == 0) {
    if (!authenticated_) {
      ReplyHttpCode(401);
    } else {
      if (mg_http_match_uri(hm_, "/sys/upload-index")) {
        mg_http_upload(c_, hm_, &mg_fs_littlefs, "/public/index.html.gz", 300000);
        // Uploading requires flash erasure, during which all interrupts have to be disabled
        // So let's pause for a while here so that other tasks can do their work
        vTaskDelay(10);
      } else if (mg_http_match_uri(hm_, "/sys/change-password")) {
        size_t len = hm_->body.len;
        if (hm_->body.len > 0 && len <= sizeof(correct_password_) - 1) {
          memcpy(correct_password_, hm_->body.ptr, hm_->body.len);
          correct_password_[len] = 0;
          rtrim(correct_password_);
          SavePassword();
          ReplyHttpCode(200);
        } else {
          ReplyHttpCode(400);
        }
      } else {
        ReplyHttpCode(404);
      }
    }
  } else {
    struct mg_http_serve_opts opts = {.root_dir = "/public", .extra_headers = extra_headers_, .fs = &mg_fs_littlefs};
    mg_http_serve_dir(c_, hm_, &opts);
  }

  c_->is_resp = 0;
  c_ = nullptr;
  hm_ = nullptr;
}

void HttpHandler::Authenticate() {
  char user[10], passwd[20];
  if (correct_password_[0] == 0) {
    authenticated_ = true;
  } else {
    mg_http_creds(hm_, user, sizeof(user), passwd, sizeof(passwd));
    authenticated_ = !strcmp(passwd, correct_password_);
  }
}

void HttpHandler::LoadPassword() {
  lfs_file f;
  memset(correct_password_, 0, sizeof(correct_password_));
  int open_ret = lfs_file_open(&lfs_instance, &f, "/password.txt", LFS_O_RDONLY);
  if (open_ret < 0) {
    return;
  }
  int read = lfs_file_read(&lfs_instance, &f, correct_password_, sizeof(correct_password_) - 1);
  if (read >= 0) {
    // ensure string is correctly terminated
    correct_password_[read] = 0;
  }
  lfs_file_close(&lfs_instance, &f);
}

void HttpHandler::SavePassword() {
  lfs_file f;
  int open_ret = lfs_file_open(&lfs_instance, &f, "/password.txt", LFS_O_WRONLY | LFS_O_CREAT);
  if (open_ret < 0) {
    return;
  }
  lfs_file_write(&lfs_instance, &f, correct_password_, strlen(correct_password_) + 1);
  lfs_file_close(&lfs_instance, &f);
}

void HttpHandler::ServeQueryBattery() {
  auto batt = batteryInfoStorage.Get();
  if (batt.has_value()) {
    ServeProtobuf(&BatteryInfo_msg, &batt.value());
  } else {
    ReplyHttpCode(503);
  }
}

void HttpHandler::ServeQueryRectifier() {
  auto rect = rectifierInfoStorage.Get();
  if (rect.has_value()) {
    ServeProtobuf(&RectifierInfo_msg, &rect.value());
  } else {
    ReplyHttpCode(503);
  }
}

void HttpHandler::ServeQueryAdc() {
  uint16_t result = adc_read();
  mg_http_reply(c_, 200, extra_headers_, "%d", result);
}

bool HttpHandler::IsPost() {
  return mg_vcmp(&hm_->method, "POST") == 0;
}

bool HttpHandler::IsGet() {
  return mg_vcmp(&hm_->method, "GET") == 0;
}

void HttpHandler::ReplyHttpCode(int code) {
  mg_http_reply(c_, code, extra_headers_, "");
}
