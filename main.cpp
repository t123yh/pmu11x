#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico/unique_id.h>
#include <cstring>
#include <crc.h>
#include <FreeRTOS_IP.h>
#include "rtt/SEGGER_RTT.h"

const uint LED_PIN = 20;
const uint LED_PIN2 = 19;

extern "C"  uint8_t getPHY();

extern "C" void vIPerfInstall();

#include "ws/mongoose.h"

static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_http_match_uri(hm, "/api/hello")) {              // On /api/hello requests,
      mg_http_reply(c, 200, "", "{%Q:%d,%Q:%d}\n", "status", 1, "free_mem", xPortGetFreeHeapSize());  // Send dynamic JSON response
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
  for (;;) mg_mgr_poll(&mgr, 1000);                       // Event loop
}

void led_task_gpio(void* _) {
  vTaskDelay(10);
  while (1) {
    uint p = LED_PIN2;
    gpio_put( p, 0);
    vTaskDelay(200);
    gpio_put( p, 1);
    vTaskDelay(200);
  }

  /*
  while (1) {
    uint8_t x;
    xQueueReceive(ledq, &x, portMAX_DELAY);
    uint p = x ? LED_PIN : LED_PIN2;
    gpio_put( p, 0);
    vTaskDelay(50);
    gpio_put( p, 1);
    vTaskDelay(50);
    uint8_t g = getPHY();
    gpio_put( LED_PIN, !(g & 1));
    vTaskDelay(100);
  }
     */

}

static const uint8_t ucIPAddress[ 4 ] = { 192, 168, 1, 211 };
static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 1, 1 };

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

int main() {
  SEGGER_RTT_Init();

  bi_decl(bi_program_description("First Blink"));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  gpio_init(LED_PIN);
  gpio_init(LED_PIN2);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_set_dir(LED_PIN2, GPIO_OUT);
  gpio_put(LED_PIN, 1);
  gpio_put(LED_PIN2, 1);

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

  xTaskCreate(led_task_gpio, "GPIO_LED_TASK",  128, NULL,tskIDLE_PRIORITY, nullptr);
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
      xTaskCreate(mg_main, "MG_TASK",  1024, NULL,tskIDLE_PRIORITY + 1, nullptr);
      xTasksAlreadyCreated = pdTRUE;
    }
    gpio_put(LED_PIN, 0);
  } else if (eNetworkEvent == eNetworkDown) {
    gpio_put(LED_PIN, 1);
  }
}

extern "C" const char *pcApplicationHostnameHook() {
  return "HAHA";
}