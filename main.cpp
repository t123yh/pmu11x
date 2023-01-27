#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico/unique_id.h>
#include <cstring>
#include <crc.h>
#include <FreeRTOS_IP.h>
#include <pb_encode.h>
#include "rtt/SEGGER_RTT.h"
#include "ArduinoJson-v6.20.0.hpp"

static const uint LED_PIN_Blue = 20;
static const uint LED_PIN_Yellow = 19;

#include "ws/mongoose.h"
#include "battery.h"
#include "led.h"
#include "controller.h"

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

static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_http_match_uri(hm, "/api/battery")) {              // On /api/hello requests,
      auto batt = getCurrentBatteryInfo();
      if (batt.has_value()) {
        size_t size = 0;
        pb_get_encoded_size(&size,&BatteryInfo_msg, &batt.value());
        mg_printf(c, "HTTP/1.1 200 OK\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Content-Type: application/x-protobuf\r\n"
                     "Connection: close\r\n"
                     "Content-Length: %d\r\n"
                     "\r\n", size);
        pb_ostream_t stream = pb_ostream_from_mg(c);
        pb_encode(&stream, &BatteryInfo_msg, &batt.value());
        c->is_resp = 0;
      } else {
        mg_http_reply(c, 500, "", "");  // Send dynamic JSON response
      }
    } else {                                                // For all other URIs,
      struct mg_http_serve_opts opts = {.root_dir = "."};   // Serve files
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
  InitLed();
  BatteryInit();
  xTaskCreate(mg_main, "MG",  4096, NULL,tskIDLE_PRIORITY, nullptr);
  xTaskCreate(controllerTask, "CTRL",  256, NULL,configMAX_PRIORITIES - 1, nullptr);
  while (1) vTaskDelay(100);
}

static const uint8_t ucIPAddress[ 4 ] = { 192, 168, 1, 211 };
static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 1, 1 };

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

int main() {
  SEGGER_RTT_Init();

  bi_decl(bi_program_description("First Blink"));
  bi_decl(bi_1pin_with_name(LED_PIN_Blue, "On-board LED"));

  gpio_init(LED_PIN_Blue);
  gpio_init(LED_PIN_Yellow);
  gpio_set_dir(LED_PIN_Blue, GPIO_OUT);
  gpio_set_dir(LED_PIN_Yellow, GPIO_OUT);
  gpio_put(LED_PIN_Blue, 1);
  gpio_put(LED_PIN_Yellow, 1);

  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  uint8_t mac_addr[6];
  uint32_t crc = xcrc32(board_id.id, sizeof(board_id), 0xFFFFFFFF);
  mac_addr[0] = 0x74;
  mac_addr[1] = 0x12;
  mac_addr[2] = 0x34;
  memcpy(mac_addr + 3, &crc, 3);

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