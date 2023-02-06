#include <sys/cdefs.h>
#include <FreeRTOS.h>
#include <task.h>
#include "ws/mongoose.h"
#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <pico/binary_info/code.h>
#include <pico/unique_id.h>

#define SPI_PORT spi0

#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 0
#define PIN_CS 1
#define PIN_RST 5
#define PIN_INT 4

static uint dma_tx;
static uint dma_rx;
static dma_channel_config dma_channel_config_tx;
static dma_channel_config dma_channel_config_rx;

static inline void w5500_select(void) {
  gpio_put(PIN_CS, 0);
}

static inline void w5500_deselect(void) {
  gpio_put(PIN_CS, 1);
}

static void w5500_port_init(void) {
  // this example will use SPI0 at 5MHz
  spi_init(SPI_PORT, 40000 * 1000);

  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

  // make the SPI pins available to picotool
  bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

  // chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);

  gpio_init(PIN_RST);
  gpio_set_dir(PIN_RST, GPIO_OUT);
  gpio_put(PIN_RST, 0);
  vTaskDelay(5);
  gpio_put(PIN_RST, 1);
  vTaskDelay(5);

  gpio_init(PIN_INT);
  gpio_set_dir(PIN_INT, GPIO_IN);

  // make the SPI pins available to picotool
  bi_decl(bi_1pin_with_name(PIN_CS, "W5x00 CHIP SELECT"));
  bi_decl(bi_1pin_with_name(PIN_RST, "W5x00 CHIP RESET"));
  bi_decl(bi_1pin_with_name(PIN_INT, "W5x00 CHIP INT"));

  dma_tx = dma_claim_unused_channel(true);
  dma_channel_config_tx = dma_channel_get_default_config(dma_tx);
  channel_config_set_transfer_data_size(&dma_channel_config_tx, DMA_SIZE_8);
  channel_config_set_dreq(&dma_channel_config_tx, DREQ_SPI0_TX);

  dma_rx = dma_claim_unused_channel(true);
  dma_channel_config_rx = dma_channel_get_default_config(dma_rx);
  channel_config_set_transfer_data_size(&dma_channel_config_rx, DMA_SIZE_8);
  channel_config_set_dreq(&dma_channel_config_rx, DREQ_SPI0_RX);
 }

enum {
  W5500_CR = 0, W5500_S0 = 1, W5500_TX0 = 2, W5500_RX0 = 3
};

static void w5500_txn(uint8_t block, uint16_t addr, bool wr,
                      void *buf, size_t len) {
  uint8_t cmd[] = {(uint8_t) (addr >> 8), (uint8_t) (addr & 255),
                   (uint8_t) ((block << 3) | (wr ? 4 : 0))};
  w5500_select();
  spi_write_blocking(SPI_PORT, cmd, sizeof(cmd));
  if (wr) {
    if (len > 2) {
      uint8_t dummy_data;

      channel_config_set_read_increment(&dma_channel_config_tx, true);
      channel_config_set_write_increment(&dma_channel_config_tx, false);
      dma_channel_configure(dma_tx, &dma_channel_config_tx,
                            &spi_get_hw(SPI_PORT)->dr, // write address
                             buf,                      // read address
                            len,                       // element count (each element is of size transfer_data_size)
                            false);                    // don't start yet

      channel_config_set_read_increment(&dma_channel_config_rx, false);
      channel_config_set_write_increment(&dma_channel_config_rx, false);
      dma_channel_configure(dma_rx, &dma_channel_config_rx,
                            &dummy_data,               // write address
                            &spi_get_hw(SPI_PORT)->dr, // read address
                            len,                       // element count (each element is of size transfer_data_size)
                            false);                    // don't start yet

      dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));
      dma_channel_wait_for_finish_blocking(dma_rx);
    } else {
      spi_write_blocking(SPI_PORT, buf, len);
    }
  } else {
    if (len > 2) {
      uint8_t dummy_data = 0xFF;

      channel_config_set_read_increment(&dma_channel_config_tx, false);
      channel_config_set_write_increment(&dma_channel_config_tx, false);
      dma_channel_configure(dma_tx, &dma_channel_config_tx,
                            &spi_get_hw(SPI_PORT)->dr, // write address
                            &dummy_data,               // read address
                            len,                       // element count (each element is of size transfer_data_size)
                            false);                    // don't start yet

      channel_config_set_read_increment(&dma_channel_config_rx, false);
      channel_config_set_write_increment(&dma_channel_config_rx, true);
      dma_channel_configure(dma_rx, &dma_channel_config_rx,
                            buf,                      // write address
                            &spi_get_hw(SPI_PORT)->dr, // read address
                            len,                       // element count (each element is of size transfer_data_size)
                            false);                    // don't start yet

      dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));
      dma_channel_wait_for_finish_blocking(dma_rx);
    } else {
      spi_write_read_blocking(SPI_PORT, buf, buf, len);
    }
  }
  w5500_deselect();
}

static void w5500_wn(uint8_t block, uint16_t addr, void *buf, size_t len) {
  w5500_txn(block, addr, true, buf, len);
}

static void w5500_w1(uint8_t block, uint16_t addr, uint8_t val) {
  w5500_wn(block, addr, &val, 1);
}

static void w5500_w2(uint8_t block, uint16_t addr, uint16_t val) {
  uint8_t buf[2] = {(uint8_t) (val >> 8), (uint8_t) (val & 255)};
  w5500_wn(block, addr, buf, sizeof(buf));
}

static void w5500_rn(uint8_t block, uint16_t addr, void *buf, size_t len) {
  w5500_txn(block, addr, false, buf, len);
}

static uint8_t w5500_r1(uint8_t block, uint16_t addr) {
  uint8_t r = 0;
  w5500_rn(block, addr, &r, 1);
  return r;
}

static uint16_t w5500_r2(uint8_t block, uint16_t addr) {
  uint8_t buf[2] = {0, 0};
  w5500_rn(block, addr, buf, sizeof(buf));
  return (uint16_t) ((buf[0] << 8) | buf[1]);
}

static size_t w5500_rx_dma(void *buf, size_t buflen, struct mip_if *ifp) {
  uint16_t r = 0, n = 0, len = (uint16_t) buflen, n2;     // Read recv len
  while ((n2 = w5500_r2(W5500_S0, 0x26)) > n) n = n2;  // Until it is stable
  // printf("RSR: %d\n", (int) n);
  if (n > 0) {
    uint16_t ptr = w5500_r2(W5500_S0, 0x28);  // Get read pointer
    n = w5500_r2(W5500_RX0, ptr);             // Read frame length
    if (n <= len + 2 && n > 1) {
      r = (uint16_t) (n - 2);
      w5500_rn(W5500_RX0, (uint16_t) (ptr + 2), buf, r);
    }
    w5500_w2(W5500_S0, 0x28, (uint16_t) (ptr + n));  // Advance read pointer
    w5500_w1(W5500_S0, 1, 0x40);                     // Sock0 CR -> RECV
    // printf("  RX_RD: tot=%u n=%u r=%u\n", n2, n, r);
  }
  return r;
}

static size_t w5500_tx_dma(const void *buf, size_t buflen, struct mip_if *ifp) {
  uint16_t n = 0, len = (uint16_t) buflen;
  while (n < len) n = w5500_r2(W5500_S0, 0x20);      // Wait for space
  uint16_t ptr = w5500_r2(W5500_S0, 0x24);           // Get write pointer
  w5500_wn(W5500_TX0, ptr, (void *) buf, len);       // Write data
  w5500_w2(W5500_S0, 0x24, (uint16_t) (ptr + len));  // Advance write pointer
  w5500_w1(W5500_S0, 1, 0x20);                       // Sock0 CR -> SEND
  for (int i = 0; i < 40; i++) {
    uint8_t ir = w5500_r1(W5500_S0, 2);  // Read S0 IR
    if (ir == 0) continue;
    // printf("IR %d, len=%d, free=%d, ptr %d\n", ir, (int) len, (int) n, ptr);
    w5500_w1(W5500_S0, 2, ir);  // Write S0 IR: clear it!
    if (ir & 8) len = 0;           // Timeout. Report error
    if (ir & (16 | 8)) break;      // Stop on SEND_OK or timeout
  }
  return len;
}

static bool w5500_init_dma(struct mip_if *ifp) {
  w5500_port_init();
  w5500_w1(W5500_CR, 0, 0x80);     // Reset chip: CR -> 0x80
  w5500_w1(W5500_CR, 0x2e, 0);     // CR PHYCFGR -> reset
  w5500_w1(W5500_CR, 0x2e, 0xf8);  // CR PHYCFGR -> set
  w5500_wn(W5500_CR, 9, ifp->mac, 6);      // Set source MAC
  w5500_w1(W5500_S0, 0x1e, 16);          // Sock0 RX buf size
  w5500_w1(W5500_S0, 0x1f, 16);          // Sock0 TX buf size
  w5500_w1(W5500_S0, 0, 4 | (1 << 7));              // Sock0 MR -> MACRAW
  w5500_w1(W5500_S0, 1, 1);              // Sock0 CR -> OPEN
  return w5500_r1(W5500_S0, 3) == 0x42;  // Sock0 SR == MACRAW
}

static bool w5500_up_dma(struct mip_if *ifp) {
  uint8_t phycfgr = w5500_r1(W5500_CR, 0x2e);
  return phycfgr & 1;  // Bit 0 of PHYCFGR is LNK (0 - down, 1 - up)
}

struct mip_driver mip_driver_w5500_dma = {w5500_init_dma, w5500_tx_dma, w5500_rx_dma, w5500_up_dma};