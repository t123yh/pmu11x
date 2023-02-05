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
#include "controller/config.h"
#include "controller/http.h"

static HttpHandler http_handler;

static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
    auto *hm = (struct mg_http_message *) ev_data;
    mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST\r\n", "");
    // http_handler.HandleRequest(c, hm);
  }
}

void mg_main(void *_) {
  struct mg_mgr mgr;
  mg_mgr_init(&mgr);                                      // Init manager
  mg_http_listen(&mgr, "http://0.0.0.0:8000", fn, &mgr);  // Setup listener
  for (;;) mg_mgr_poll(&mgr, 10);                       // Event loop
}

static uint8_t dev_mac_addr[6];

void init_task(void *_) {
  lfsInit();
  lfs_remove(&lfs_instance, "/power_config.pb");
  lfs_mkdir(&lfs_instance, "/public");
  LoadConfigFromFilesystem();
  http_handler.LoadPassword();

  auto net = networkConfigItem.Get();
  FreeRTOS_IPInit(net.static_ip.addr,
                  net.static_ip.netmask,
                  net.static_ip.gateway,
                  net.static_ip.dns_server,
                  dev_mac_addr);

  ds1302Init();
  InitLed();
  BatteryInit();
  RectifierInit();
  SysBlueLed.mode = Led::REPEAT;
  SysBlueLed.period = 500;
  xTaskCreate(timeWork, "NTP", 512, NULL, tskIDLE_PRIORITY, nullptr);
  xTaskCreate(mg_main, "MG", 4096, NULL, tskIDLE_PRIORITY, nullptr);
  xTaskCreate(controllerTask, "CTRL", 512, NULL, configMAX_PRIORITIES - 1, nullptr);
  while (1) {
    vTaskDelay(5000);
  }
}

int main() {
  SEGGER_RTT_Init();

  bi_decl(bi_program_description("First Blink"));
  bi_decl(bi_1pin_with_name(LED_PIN_Blue, "On-board LED"));

  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  uint32_t crc = lfs_crc(0xFFFFFFFF, board_id.id, sizeof(board_id));
  dev_mac_addr[0] = 0x74;
  dev_mac_addr[1] = 0x12;
  dev_mac_addr[2] = 0x34;
  memcpy(dev_mac_addr + 3, &crc, 3);

  adc_init();
  adc_gpio_init(28);
  adc_select_input(2);


  xTaskCreate(init_task, "INIT", 512, NULL, tskIDLE_PRIORITY, nullptr);
  vTaskStartScheduler();

  for (;;);
}

extern "C" void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
  static BaseType_t xTasksAlreadyCreated = pdFALSE;

  /* Both eNetworkUp and eNetworkDown events can be processed here. */
  if (eNetworkEvent == eNetworkUp) {
    /* Create the tasks that use the TCP/IP stack if they have not already
    been created. */
    if (xTasksAlreadyCreated == pdFALSE) {
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

static char hostname_buffer[13];
extern "C" const char *pcApplicationHostnameHook() {
  auto net = networkConfigItem.Get();
  memcpy(hostname_buffer, net.dhcp.hostname, 13);
  return hostname_buffer;
}

eDHCPCallbackAnswer_t xApplicationDHCPHook(eDHCPCallbackPhase_t eDHCPPhase,
                                           uint32_t ulIPAddress) {
  auto net = networkConfigItem.Get();
  if (net.has_dhcp) {
    return eDHCPContinue;
  } else {
    return eDHCPUseDefaults;
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char * pcTaskName) {
  portDISABLE_INTERRUPTS();
  while (1) asm("nop");
}