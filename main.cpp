#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico/unique_id.h>
#include <cstring>
#include <crc.h>
#include <FreeRTOS_IP.h>
#include <pb_encode.h>
#include <hardware/adc.h>
#include "rtt/SEGGER_RTT.h"

static const uint LED_PIN_Blue = 20;
static const uint LED_PIN_Yellow = 19;

#include "ws/mongoose.h"
#include "controller/battery.h"
#include "led.h"
#include "controller/controller.h"
#include "time/time.h"
#include "time/ds1302.h"
#include "littlefs/lfs_util.h"
#include "controller/rectifier.h"
#include "fs.h"

static bool pb_mg_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count)
{
mg_connection *dest = (mg_connection*)stream->state;
mg_send(dest, buf, count);
return true;
}

pb_ostream_t pb_ostream_from_mg(mg_connection* conn)
{
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

static void dateHeader(char* buf, int len) {
  const char *weekday_names[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
  const char *month_names[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
  tm timeinfo{};
  ds1302GetTime(&timeinfo);
  snprintf(buf, len, "Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n",
         weekday_names[timeinfo.tm_wday],
         timeinfo.tm_mday,
         month_names[timeinfo.tm_mon],
         timeinfo.tm_year + 1900,
         timeinfo.tm_hour,
         timeinfo.tm_min,
         timeinfo.tm_sec);
}

static char date_header[38];
static void serveProtobuf(struct mg_connection* c, const pb_msgdesc_t *fields, const void *src_struct) {
  size_t size = 0;
  pb_get_encoded_size(&size, fields, src_struct);
  mg_printf(c, "HTTP/1.1 200 OK\r\n"
               "Cache-Control: no-cache\r\n"
               "Content-Type: application/x-protobuf\r\n"
               "Connection: close\r\n"
               "Content-Length: %d\r\n"
               "%s"
               "\r\n", size, date_header);
  pb_ostream_t stream = pb_ostream_from_mg(c);
  pb_encode(&stream, fields, src_struct);
  c->is_resp = 0;
}

extern bool ce;
static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  dateHeader(date_header, sizeof(date_header));
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_http_match_uri(hm, "/query/battery")) {              // On /api/hello requests,
      auto batt = batteryInfoStorage.Get();
      if (batt.has_value()) {
        serveProtobuf(c, &BatteryInfo_msg, &batt.value());
      } else {
        mg_http_reply(c, 500, date_header, "");  // Send dynamic JSON response
      }
    } else if (mg_http_match_uri(hm, "/query/rectifier")) {              // On /api/hello requests,
        auto batt = rectifierInfoStorage.Get();
        if (batt.has_value()) {
          serveProtobuf(c, &RectifierInfo_msg, &batt.value());
        } else {
          mg_http_reply(c, 500, date_header, "");  // Send dynamic JSON response
        }
    } else if (mg_http_match_uri(hm, "/ctrl/rectifier-off")) {              // On /api/hello requests,
      ce = false;
      mg_http_reply(c, 200, date_header, "");  // Send dynamic JSON response
    } else if (mg_http_match_uri(hm, "/ctrl/rectifier-on")) {              // On /api/hello requests,
      ce = true;
      mg_http_reply(c, 200, date_header, "");  // Send dynamic JSON response
    } else if (mg_http_match_uri(hm, "/api/bus")) {
      uint16_t result = adc_read();
      mg_http_reply(c, 200, date_header, "%d", result);  // Send dynamic JSON response
    } else {                                                // For all other URIs,
      struct mg_http_serve_opts opts = {.root_dir = ".", .extra_headers = date_header, .fs = &mg_fs_littlefs};   // Serve files
      mg_http_serve_dir(c, hm, &opts);                      // From root_dir
    }
  }
}

void mg_main(void* _) {
  struct mg_mgr mgr;
  mg_mgr_init(&mgr);                                      // Init manager
  mg_http_listen(&mgr, "http://0.0.0.0:8000", fn, &mgr);  // Setup listener
  for (;;) mg_mgr_poll(&mgr, 10);                       // Event loop
}

void init_task(void* _) {
  ds1302Init();
  InitLed();
  BatteryInit();
  RectifierInit();
  SysBlueLed.mode = Led::REPEAT;
  SysBlueLed.period = 500;
  lfsInit();
  xTaskCreate(timeWork, "NTP", 256, NULL, tskIDLE_PRIORITY, nullptr);
  xTaskCreate(mg_main, "MG",  4096, NULL,tskIDLE_PRIORITY, nullptr);
  xTaskCreate(controllerTask, "CTRL",  256, NULL,configMAX_PRIORITIES - 1, nullptr);
  while (1) {
    vTaskDelay(5000);
  }
}

static const uint8_t ucIPAddress[ 4 ] = { 192, 168, 1, 211 };
static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 1, 1 };

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 192,168,1,1 };

int main() {
  SEGGER_RTT_Init();

  bi_decl(bi_program_description("First Blink"));
  bi_decl(bi_1pin_with_name(LED_PIN_Blue, "On-board LED"));

  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  uint8_t mac_addr[6];
  uint32_t crc = lfs_crc(0xFFFFFFFF, board_id.id, sizeof(board_id));
  mac_addr[0] = 0x74;
  mac_addr[1] = 0x12;
  mac_addr[2] = 0x34;
  memcpy(mac_addr + 3, &crc, 3);

  adc_init();
  adc_gpio_init(28);
  adc_select_input(2);

  FreeRTOS_IPInit( ucIPAddress,
                   ucNetMask,
                   ucGatewayAddress,
                   ucDNSServerAddress,
                   mac_addr);

  xTaskCreate(init_task, "INIT",  128, NULL,tskIDLE_PRIORITY, nullptr);
  vTaskStartScheduler();

  for( ;; );
}

extern "C" void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
  static BaseType_t xTasksAlreadyCreated = pdFALSE;

  /* Both eNetworkUp and eNetworkDown events can be processed here. */
  if( eNetworkEvent == eNetworkUp )
  {
    /* Create the tasks that use the TCP/IP stack if they have not already
    been created. */
    if( xTasksAlreadyCreated == pdFALSE )
    {
      /*
       * For convenience, tasks that use FreeRTOS-Plus-TCP can be created here
       * to ensure they are not created before the network is usable.
       */
      xTasksAlreadyCreated = pdTRUE;
    }
    // gpio_put(LED_PIN_Blue, 0);
  } else if (eNetworkEvent == eNetworkDown) {
    // gpio_put(LED_PIN_Blue, 1);
  }
}

extern "C" const char *pcApplicationHostnameHook() {
  return "HAHA";
}