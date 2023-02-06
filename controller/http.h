//
// Created by 田韵豪 on 2023/2/5.
//

#ifndef PMU11X_CTRL_HTTP_H
#define PMU11X_CTRL_HTTP_H

#include "../net/mongoose.h"

class HttpHandler {
private:
  char extra_headers_[200];
  void MakeExtraHeaders();
  bool authenticated_;
  void Authenticate();
  mg_connection* c_;
  mg_http_message* hm_;

  void ServeProtobuf(const pb_msgdesc_t *fields, const void *src_struct);
  void ReplyHttpCode(int code);
  bool IsGet();
  bool IsPost();

  void ServeQueryBattery();
  void ServeQueryRectifier();
  void ServeQueryAdc();

  char correct_password_[20];

public:
  HttpHandler() = default;
  void HandleRequest(mg_connection* c, mg_http_message* msg);

  void LoadPassword();
  void SavePassword();
};

#endif //PMU11X_CTRL_HTTP_H
