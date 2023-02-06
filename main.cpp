#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico/unique_id.h>
#include <cstring>
#include <crc.h>
#include <pb_encode.h>
#include <hardware/adc.h>
#include <ctime>
#include <hardware/watchdog.h>
#include "rtt/SEGGER_RTT.h"

static const uint LED_PIN_Blue = 20;
static const uint LED_PIN_Yellow = 19;

#include "ws/mongoose.h"
#include "controller/battery.h"
#include "led.h"
#include "controller/controller.h"
#include "time/ds1302.h"
#include "littlefs/lfs_util.h"
#include "controller/rectifier.h"
#include "fs.h"
#include "controller/config.h"
#include "controller/http.h"
#include "time/sntp.h"

static HttpHandler http_handler;

static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
    auto *hm = (struct mg_http_message *) ev_data;
    http_handler.HandleRequest(c, hm);
  }
}

extern "C" struct mip_driver mip_driver_w5500_dma;

static void FillMacAddress(uint8_t* dev_mac_addr) {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  uint32_t crc = lfs_crc(0xFFFFFFFF, board_id.id, sizeof(board_id));
  dev_mac_addr[0] = 0x74;
  dev_mac_addr[1] = 0x12;
  dev_mac_addr[2] = 0x34;
  memcpy(dev_mac_addr + 3, &crc, 3);
}

void mg_main(void *_) {
  auto net = networkConfigItem.Get();
  struct mip_if mif = {.mac = {},
      .driver = &mip_driver_w5500_dma};
  struct mg_mgr mgr;
  FillMacAddress(mif.mac);
  mg_mgr_init(&mgr);
  if (net.which_ip == NetworkConfig_static_addr_tag) {
    memcpy(&mif.ip, net.ip.static_addr.addr, 4);
    memcpy(&mif.gw, net.ip.static_addr.gateway, 4);
    memcpy(&mif.mask, net.ip.static_addr.netmask, 4);
    char* dns_server = (char*)pvPortMalloc(25);
    mg_snprintf(dns_server, 25, "udp://%d.%d.%d.%d:53",
                net.ip.static_addr.dns_server[0],
                net.ip.static_addr.dns_server[1],
                net.ip.static_addr.dns_server[2],
                net.ip.static_addr.dns_server[3]);
    mgr.dns4.url = dns_server;
  }
  mip_init(&mgr, &mif);
  AddSntpCallback(&mgr);
  mg_http_listen(&mgr, "http://0.0.0.0:8000", fn, &mgr);
  for (;;) mg_mgr_poll(&mgr, 5);
}

void init_task(void *_) {
  lfsInit();
  lfs_mkdir(&lfs_instance, "/public");
  LoadConfigFromFilesystem();
  http_handler.LoadPassword();


  ds1302Init();
  InitLed();
  BatteryInit();
  RectifierInit();
  SysBlueLed.mode = Led::REPEAT;
  SysBlueLed.period = 500;
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

  adc_init();
  adc_gpio_init(28);
  adc_select_input(2);
  watchdog_enable(2000, true);

  xTaskCreate(init_task, "INIT", 512, NULL, tskIDLE_PRIORITY, nullptr);
  vTaskStartScheduler();

  for (;;);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char * pcTaskName) {
  portDISABLE_INTERRUPTS();
  while (1) asm("nop");
}