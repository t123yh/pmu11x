#include <sys/cdefs.h>
#include <FreeRTOS.h>
#include <task.h>
#include "w5500.h"
#include "semphr.h"
#include "FreeRTOS_IP.h"
#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <pico/binary_info/code.h>
#include <pico/unique_id.h>
#include <hardware/structs/rosc.h>

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
SemaphoreHandle_t chip_sem;

static inline void w5500_select(void) {
  gpio_put(PIN_CS, 0);
}

static inline void w5500_deselect(void) {
  gpio_put(PIN_CS, 1);
}

#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03

uint8_t WIZCHIP_READ(uint32_t AddrSel) {
  uint8_t ret;
  uint8_t spi_data[3];
  AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
  spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
  spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
  spi_data[2] = (AddrSel & 0x000000FF) >> 0;

  w5500_select();
  spi_write_blocking(SPI_PORT, spi_data, sizeof(spi_data));
  spi_read_blocking(SPI_PORT, 0xFF, &ret, 1);
  w5500_deselect();
  return ret;
}

void WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb) {
  uint8_t spi_data[4];
  AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
  spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
  spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
  spi_data[2] = (AddrSel & 0x000000FF) >> 0;
  spi_data[3] = wb;

  w5500_select();
  spi_write_blocking(SPI_PORT, spi_data, sizeof(spi_data));
  w5500_deselect();
}

uint16_t getSn_TX_FSR(uint8_t sn) {
  uint16_t val = 0, val1 = 0;

  do {
    val1 = WIZCHIP_READ(Sn_TX_FSR(sn));
    val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
    if (val1 != 0) {
      val = WIZCHIP_READ(Sn_TX_FSR(sn));
      val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
    }
  } while (val != val1);
  return val;
}


uint16_t getSn_RX_RSR(uint8_t sn) {
  uint16_t val = 0, val1 = 0;

  do {
    val1 = WIZCHIP_READ(Sn_RX_RSR(sn));
    val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
    if (val1 != 0) {
      val = WIZCHIP_READ(Sn_RX_RSR(sn));
      val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
    }
  } while (val != val1);
  return val;
}


static TaskHandle_t w5500IntHandle;

static void w5500_read_dma(uint32_t AddrSel, uint8_t *pBuf, uint16_t len) {
  uint8_t spi_data[3];
  AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
  spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
  spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
  spi_data[2] = (AddrSel & 0x000000FF) >> 0;

  w5500_select();
  spi_write_blocking(SPI_PORT, spi_data, sizeof(spi_data));
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
                        pBuf,                      // write address
                        &spi_get_hw(SPI_PORT)->dr, // read address
                        len,                       // element count (each element is of size transfer_data_size)
                        false);                    // don't start yet

  dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));
  dma_channel_wait_for_finish_blocking(dma_rx);
  w5500_deselect();
}

static void w5500_write_dma(uint32_t AddrSel, const uint8_t *pBuf, uint16_t len) {
  uint8_t spi_data[3];
  AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
  spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
  spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
  spi_data[2] = (AddrSel & 0x000000FF) >> 0;

  w5500_select();
  spi_write_blocking(SPI_PORT, spi_data, sizeof(spi_data));
  uint8_t dummy_data;

  channel_config_set_read_increment(&dma_channel_config_tx, true);
  channel_config_set_write_increment(&dma_channel_config_tx, false);
  dma_channel_configure(dma_tx, &dma_channel_config_tx,
                        &spi_get_hw(SPI_PORT)->dr, // write address
                        pBuf,                      // read address
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
  w5500_deselect();
}

void WIZCHIP_READ_BUF(uint32_t AddrSel, uint8_t *pBuf, uint16_t len) {
  w5500_read_dma(AddrSel, pBuf, len);
}

void WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t *pBuf, uint16_t len) {
  w5500_write_dma(AddrSel, pBuf, len);
}

static TaskHandle_t sendingTaskHandle;

static void w5500_interrupt_callback(uint gpio, uint32_t event_mask) {
  BaseType_t woken = pdFALSE;
  if (w5500IntHandle) {
    vTaskNotifyGiveFromISR(w5500IntHandle, &woken);
  }
  portYIELD_FROM_ISR(woken);
}


_Noreturn static void w5500_interrupt_task(void *pvParameters) {
  xSemaphoreTake(chip_sem, portMAX_DELAY);
  uint8_t ir_tmp = getSn_IR(0);
  setSn_IR(0, ir_tmp);
  setSIMR(1); // enable socket 0 interrupt
  xSemaphoreGive(chip_sem);
  gpio_set_irq_enabled_with_callback(PIN_INT, GPIO_IRQ_EDGE_FALL, true, &w5500_interrupt_callback);

  for (;;) {
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(100U));

    xSemaphoreTake(chip_sem, portMAX_DELAY);

    while (gpio_get(PIN_INT) == 0) {
      uint8_t ir = getSn_IR(0);
      setSn_IR(0, ir);
      if (ir & Sn_IR_SENDOK) {
        if (sendingTaskHandle) {
          xTaskNotifyGive(sendingTaskHandle);
          sendingTaskHandle = NULL;
        }
      }
      if (ir & Sn_IR_RECV) {
        // Process until end
        while (getSn_RX_RSR(0) != 0) {
          volatile uint16_t rsr = getSn_RX_RSR(0);
          // Get current data pointer
          uint16_t ptr = getSn_RX_RD(0);

          // Read data packet length from buffer
          uint16_t len;
          WIZCHIP_READ_BUF(((uint32_t) ptr << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3), (uint8_t *) &len, 2);
          // len is big-endian, we have to convert it to little; 2 bytes are the space the length itself
          len = __builtin_bswap16(len) - 2;
          ptr += 2;

          // Allocate buffer
          NetworkBufferDescriptor_t *pxDescriptor = pxGetNetworkBufferWithDescriptor(len, 0);
          if (pxDescriptor != NULL) {
            // Transfer actual data
            w5500_read_dma(((uint32_t) ptr << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3), pxDescriptor->pucEthernetBuffer,
                           len);
            pxDescriptor->xDataLength = len;
          }

          // Increase pointer
          ptr += len;
          // Write back read pointer
          setSn_RX_RD(0, ptr);
          // Notify device
          setSn_CR(0, Sn_CR_RECV);

          if (pxDescriptor != NULL) {
            if (eConsiderFrameForProcessing(pxDescriptor->pucEthernetBuffer)
                == eProcessBuffer) {
              IPStackEvent_t xRxEvent;
              xRxEvent.eEventType = eNetworkRxEvent;
              xRxEvent.pvData = (void *) pxDescriptor;
              if (xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE) {
                vReleaseNetworkBufferAndDescriptor(pxDescriptor);
                iptraceETHERNET_RX_EVENT_LOST();
              } else {
                iptraceNETWORK_INTERFACE_RECEIVE();
              }
            } else {
              vReleaseNetworkBufferAndDescriptor(pxDescriptor);
            }
          }
        }
      }
    }

    uint8_t pcfg = getPHYCFGR();
    if (!(pcfg & PHY_LINK_ON)) {
      FreeRTOS_NetworkDown();
    }

    xSemaphoreGive(chip_sem);
  }
}

void w5500_init(void) {
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
  chip_sem = xSemaphoreCreateMutex();

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

  xSemaphoreTake(chip_sem, portMAX_DELAY);
  setSHAR(ipLOCAL_MAC_ADDRESS);
  for (int i = 1; i < 8; i++) {
    setSn_RXBUF_SIZE(i, 0);
    setSn_TXBUF_SIZE(i, 0);
  }
  setSn_RXBUF_SIZE(0, 16);
  setSn_TXBUF_SIZE(0, 16);
  setSIMR(0); // disable socket 0 interrupt for now, enable it later
  setIMR(0);
  setSn_MR(0, Sn_MR_MACRAW | Sn_MR_MFEN);
  setSn_MR(0, Sn_MR_MACRAW);
  setSn_CR(0, Sn_CR_OPEN);
  setSn_IMR(0, Sn_IR_RECV | Sn_IR_SENDOK);
  xSemaphoreGive(chip_sem);

  xTaskCreate(w5500_interrupt_task, "w5500_int", 128, NULL, configMAX_PRIORITIES - 1, &w5500IntHandle);
}

BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t *const pxDescriptor,
                                   BaseType_t xReleaseAfterSend) {
  BaseType_t transmitted = pdFALSE;
  xSemaphoreTake(chip_sem, portMAX_DELAY);
  size_t data_len = pxDescriptor->xDataLength;
  // Check if remaining space is enough to contain data
  if (getSn_TX_FSR(0) >= data_len) {
    uint16_t ptr = getSn_TX_WR(0);
    w5500_write_dma(((uint32_t) ptr << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3), pxDescriptor->pucEthernetBuffer,
                    data_len);
    setSn_TX_WR(0, ptr + data_len);
    setSn_CR(0, Sn_CR_SEND);
    iptraceNETWORK_INTERFACE_TRANSMIT();
    transmitted = pdTRUE;
  }
  if (xReleaseAfterSend != pdFALSE) {
    vReleaseNetworkBufferAndDescriptor(pxDescriptor);
  }
  sendingTaskHandle = xTaskGetCurrentTaskHandle();
  xSemaphoreGive(chip_sem);

  // Wait for SENDOK interrupt
  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

  return transmitted;
}

BaseType_t xNetworkInterfaceInitialise(void) {
  static bool inited = false;
  if (!inited) {
    w5500_init();
    inited = true;
  }

  xSemaphoreTake(chip_sem, portMAX_DELAY);
  uint8_t pcfg = getPHYCFGR();
  xSemaphoreGive(chip_sem);

  return (pcfg & PHYCFGR_LNK_ON) ? pdPASS : pdFAIL;
}

uint32_t xor64() {
  static uint32_t x = 123456789, y = 362436069;
  x ^= rosc_hw->randombit;
  uint32_t t = (x ^ (x << 10));
  x = y;
  return y = (y ^ (y >> 10)) ^ (t ^ (t >> 13));
}

BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber) {
  *pulNumber = xor64();
  return pdTRUE;
}

extern uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                                   uint16_t usSourcePort,
                                                   uint32_t ulDestinationAddress,
                                                   uint16_t usDestinationPort) {
  (void) ulSourceAddress;
  (void) usSourcePort;
  (void) ulDestinationAddress;
  (void) usDestinationPort;

  return xor64();
}
