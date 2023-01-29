//
// Created by 田韵豪 on 2023/1/28.
//

#include "ds1302.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ds1302.pio.h>
#include <pico/types.h>
#include <hardware/pio.h>
#include <hardware/structs/clocks.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <ctime>

static const uint pin_sclk = 24;
static const uint pin_io = 25;
static const uint pin_ce = 26;
static const PIO pio = pio1;
static uint sm;

static uint dma_ch;
static dma_channel_config dma_config;
static TaskHandle_t pendingTask;
static SemaphoreHandle_t ds1302Mutex;

static void irqHandler() {
  BaseType_t woken;
  pio_interrupt_clear(pio, 0);
  if (pendingTask) {
    xTaskNotifyFromISR(pendingTask, 0, eSetValueWithOverwrite, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

static void ds1302PioInit() {
  sm = pio_claim_unused_sm(pio, true);
  uint offset = pio_add_program(pio, &ds1302_program);
  pio_sm_config c = ds1302_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pin_io, 1);
  sm_config_set_in_pins(&c, pin_io);
  sm_config_set_set_pins(&c, pin_io, 1);
  sm_config_set_sideset_pins(&c, pin_sclk);
  sm_config_set_out_shift(&c, true, true, 8);
  sm_config_set_in_shift(&c, true, true, 8);
  sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 500000);
  pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_sclk) | (1u << pin_io));
  pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin_sclk, (1u << pin_sclk) | (1u << pin_io));

  pio_gpio_init(pio, pin_sclk);
  pio_gpio_init(pio, pin_io);

  gpio_init(pin_ce);
  gpio_put(pin_ce, false);
  gpio_set_dir(pin_ce, GPIO_OUT);

  pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
  irq_set_exclusive_handler(PIO1_IRQ_0, irqHandler);
  irq_set_enabled(PIO1_IRQ_0, true);

  pio_sm_init(pio, sm, offset, &c);

  dma_ch = dma_claim_unused_channel(true);
  dma_config = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);

  pio_sm_set_enabled(pio, sm, true);
}

static void ds1302_read(uint8_t reg, uint8_t *dst, uint8_t len) {
  pendingTask = xTaskGetCurrentTaskHandle();
  gpio_put(pin_ce, true);
  auto* txfifo = (io_rw_8 *) &pio->txf[sm];
  // TX FIFO is initially empty, fill it without checking
  *txfifo = 0;   // write len
  *txfifo = reg;
  *txfifo = len; // read len
  io_ro_8 *rxfifo = (io_ro_8 *) &pio->rxf[sm] + 3;
  channel_config_set_read_increment(&dma_config, false);
  channel_config_set_write_increment(&dma_config, true);
  channel_config_set_dreq(&dma_config, pio_get_dreq(pio, sm, false));
  dma_channel_configure(dma_ch, &dma_config, dst, rxfifo, len, true);
  xTaskNotifyWait(0, 0xFFFFFFFF, nullptr, portMAX_DELAY);
  dma_channel_wait_for_finish_blocking(dma_ch);
  gpio_put(pin_ce, false);
}

static void ds1302_write_one(uint8_t reg, uint8_t dat) {
  pendingTask = xTaskGetCurrentTaskHandle();
  gpio_put(pin_ce, true);
  auto* txfifo = (io_rw_8 *) &pio->txf[sm];
  *txfifo = 1; // write len = 1
  *txfifo = reg;
  *txfifo = dat;
  *txfifo = 0; // read len = 0
  xTaskNotifyWait(0, 0xFFFFFFFF, nullptr, portMAX_DELAY);
  gpio_put(pin_ce, false);
}

// The actual capacity of src must be one byte longer than len!!
static void ds1302_write(uint8_t reg, uint8_t* src, uint8_t len) {
  pendingTask = xTaskGetCurrentTaskHandle();
  gpio_put(pin_ce, true);
  auto* txfifo = (io_rw_8 *) &pio->txf[sm];
  *txfifo = len;
  *txfifo = reg;
  src[len] = 0; // The last byte is read length, set to 0
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_write_increment(&dma_config, false);
  channel_config_set_dreq(&dma_config, pio_get_dreq(pio, sm, true));
  dma_channel_configure(dma_ch, &dma_config, txfifo, src, len + 1, true);
  xTaskNotifyWait(0, 0xFFFFFFFF, nullptr, portMAX_DELAY);
  gpio_put(pin_ce, false);
}

void ds1302Init() {
  ds1302Mutex = xSemaphoreCreateMutex();
  ds1302PioInit();
  // Disable write protect
  ds1302_write_one(0x8E, 0);
  // Enable trickle charger
  ds1302_write_one(0x90, 0b10100101);
  // Enable write protect
  ds1302_write_one(0x8E, 0);
}

static inline uint8_t bin2bcd(uint8_t x) {
  return (((x) / 10) << 4) + (x) % 10;
}

static inline uint8_t bcd2bin(uint8_t x) {
  return ((x) & 0x0f) + ((x) >> 4) * 10;
}

void ds1302SetTime(const struct tm* t) {
  uint8_t buf[] = {
      bin2bcd(t->tm_sec),
      bin2bcd(t->tm_min),
      bin2bcd(t->tm_hour),
      bin2bcd(t->tm_mday),
      bin2bcd(t->tm_mon + 1),
      bin2bcd((t->tm_wday + 6) % 7 + 1),
      bin2bcd(t->tm_year % 100),
      0, // WP
      0
  };

  if (xSemaphoreTake(ds1302Mutex, portMAX_DELAY) == pdFALSE) {
    return;
  }
  // Disable write protect
  ds1302_write_one(0x8E, 0);
  // Write data
  ds1302_write(0xBE, buf, 8);
  xSemaphoreGive(ds1302Mutex);
}

void ds1302GetTime(struct tm* t) {
  if (xSemaphoreTake(ds1302Mutex, portMAX_DELAY) == pdFALSE) {
    return;
  }
  uint8_t buf[7];
  // Burst read data
  ds1302_read(0xBF, buf, 7);
  // If clock is not halt
  if (!(buf[0] & 0x80)) {
    t->tm_sec = bcd2bin(buf[0] & 0b1111111);
    t->tm_min = bcd2bin(buf[1]);
    t->tm_hour = bcd2bin(buf[2] & 0b111111);
    t->tm_mday = bcd2bin(buf[3]);
    t->tm_mon = bcd2bin(buf[4]) - 1;
    t->tm_wday = bcd2bin(buf[5] % 7);
    t->tm_year = bcd2bin(buf[6]) + 2000;
  }
  xSemaphoreGive(ds1302Mutex);
}

/*
void __time_critical_func(ds1302_read)(uint8_t reg, uint8_t *dst,
                                                         uint8_t read_len) {
  gpio_put(pin_ce, true);
  uint8_t write_dat[] = {0, reg, read_len};
  io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[sm];
  io_ro_8 *rxfifo = (io_ro_8 *) &pio->rxf[sm] + 3;
  for (int i = 0; i < sizeof(write_dat); i++) {
    while (pio_sm_is_tx_fifo_full(pio, sm));
    *txfifo = write_dat[i];
  }
  for (int i = 0; i < read_len; i++) {
    while (pio_sm_is_rx_fifo_empty(pio, sm));
    dst[i] = *rxfifo;
  }
  while (!pio_interrupt_get(pio, 0));
  pio_interrupt_clear(pio, 0);
  gpio_put(pin_ce, false);
}*/

/*
void __time_critical_func(ds1302_write)(uint8_t reg, uint8_t *dst, uint8_t len) {
  gpio_put(pin_ce, true);
  io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[sm];
  while (pio_sm_is_tx_fifo_full(pio, sm));
  *txfifo = len;
  while (pio_sm_is_tx_fifo_full(pio, sm));
  *txfifo = reg;
  for (int i = 0; i < len; i++) {
    while (pio_sm_is_tx_fifo_full(pio, sm));
    *txfifo = dst[i];
  }
  while (pio_sm_is_tx_fifo_full(pio, sm));
  *txfifo = 0;
  while (!pio_interrupt_get(pio, 0));
  pio_interrupt_clear(pio, 0);
  gpio_put(pin_ce, false);
}
*/