//
// Created by 田韵豪 on 2023/1/23.
//

#include <cstdint>
#include "battery.h"
#include "led.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <pico.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>

#ifdef BAT_RS485
#define UART_TX_PIN 12
#define UART_RX_PIN 13
#define UART_NUM  uart0
#else
#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define UART_NUM uart1
#endif

const uint8_t kBatteryAddress = 0;

struct BatteryRequest {
  uint8_t address;
  uint8_t command;
  const uint8_t* req;
  size_t req_len;
  uint8_t* resp;
  size_t resp_len;
  TickType_t timeout;
};

static const BatteryRequest* currentRequest;
static uint8_t actualResponseLength;

enum BatteryCommStatus {
  IDLE,
  REQ_FRAME_HEAD,
  REQ_ADDR,
  REQ_CMD,
  REQ_DATA_LEN,
  REQ_DATA,
  REQ_CHK,
  REQ_TAIL,
  RESP_FRAME_HEAD,
  RESP_ADDR,
  RESP_CMD,
  RESP_DATA_LEN,
  RESP_DATA,
  RESP_CHK,
  RESP_TAIL,
};

static BatteryCommStatus comm;
static uint8_t dataPtr;
static TaskHandle_t waitingTask;
const uint32_t kMagicOk = 0x57A291C8;
const uint32_t kMagicErr = 0xE891CD17;

struct ShinwaChecksum {
  uint8_t chk, sum;
  void Add(uint8_t dat) {
    chk ^= dat;
    sum += dat;
  }

  uint8_t Calc() const {
    return chk ^ sum;
  }

  void Reset() {
    chk = sum = 0;
  }
};

static ShinwaChecksum checksum;

static void WriteByte(uint8_t byte) {
  uart_get_hw(UART_NUM)->dr = byte;
  checksum.Add(byte);
}

static uint8_t ReadByte() {
  uint8_t ch = uart_get_hw(UART_NUM)->dr;
  checksum.Add(ch);
  return ch;
}

static SemaphoreHandle_t batteryMutex;

static void ClearFifo() {
  while (uart_is_readable(UART_NUM)) {
    uart_getc(UART_NUM);
  }
}

static void UartEvent() {
  BaseType_t woken = pdFALSE;
  if (currentRequest) {
    while (uart_is_writable(UART_NUM)) {
      bool done = false;
      switch (comm) {
        case REQ_FRAME_HEAD:
          checksum.Reset();
          WriteByte(0x7E);
          comm = REQ_ADDR;
          break;
        case REQ_ADDR:
          WriteByte(currentRequest->address);
          comm = REQ_CMD;
          break;
        case REQ_CMD:
          WriteByte(currentRequest->command);
          comm = REQ_DATA_LEN;
          break;
        case REQ_DATA_LEN:
          WriteByte(currentRequest->req_len);
          dataPtr = 0;
          if (currentRequest->req_len == 0) {
            comm = REQ_CHK;
          } else {
            comm = REQ_DATA;
          }
          break;
        case REQ_DATA:
          if (dataPtr < currentRequest->req_len) {
            WriteByte(currentRequest->req[dataPtr]);
            dataPtr++;
          }
          if (dataPtr == currentRequest->req_len) {
            comm = REQ_CHK;
          }
          break;
        case REQ_CHK:
          WriteByte(checksum.Calc());
          comm = REQ_TAIL;
          break;
        case REQ_TAIL:
          ComGreenLed.mode = Led::TRIGGER;
          WriteByte(0x0D);
          comm = RESP_FRAME_HEAD;
          break;
        default:
          // nothing to transmit
          done = true;
          break;
      }
      if (done) {
        // clear interrupt
        hw_write_masked(&uart_get_hw(UART_NUM)->icr, 1 << UART_UARTICR_TXIC_LSB,
                        UART_UARTICR_TXIC_BITS);
        // exit while loop
        break;
      }
    }

    while (uart_is_readable(UART_NUM)) {
      bool done = false, err = false;
      if (comm == RESP_FRAME_HEAD) {
        checksum.Reset();
      }
      uint8_t chksum = checksum.Calc();
      uint8_t dat = ReadByte();
      switch (comm) {
        case RESP_FRAME_HEAD:
          if (dat == 0x7E) {
            comm = RESP_ADDR;
          } else {
            err = true;
          }
          break;
        case RESP_ADDR:
          if (dat == currentRequest->address) {
            comm = RESP_CMD;
          } else {
            err = true;
          }
          break;
        case RESP_CMD:
          if (dat == currentRequest->command) {
            comm = RESP_DATA_LEN;
          } else {
            err = true;
          }
          break;
        case RESP_DATA_LEN:
          actualResponseLength = dat;
          if (actualResponseLength == 0) {
            comm = RESP_CHK;
          } else {
            comm = RESP_DATA;
          }
          dataPtr = 0;
          break;
        case RESP_DATA:
          if (dataPtr < actualResponseLength) {
            if (dataPtr < currentRequest->resp_len) {
              currentRequest->resp[dataPtr] = dat;
            }
            dataPtr++;
          }
          if (dataPtr == actualResponseLength) {
            comm = RESP_CHK;
          }
          break;
        case RESP_CHK:
          if (chksum == dat) {
            comm = RESP_TAIL;
          } else {
            err = true;
          }
          break;
        case RESP_TAIL:
          ComYellowLed.mode = Led::TRIGGER;
          if (dat == 0x0D) {
            done = true;
          } else {
            err = true;
          }
          break;
        default:
          err = true;
          break;
      }
      if (err || done) {
        if (waitingTask) {
          uint32_t val = err ? kMagicErr : kMagicOk;
          xTaskNotifyFromISR(waitingTask, val, eSetValueWithOverwrite, &woken);
          waitingTask = nullptr;
        }
        comm = IDLE;
        ClearFifo();
        break;
      }
    }
  } else {
    // No transfer is ongoing, clear interrupts
    if (uart_get_hw(UART_NUM)->mis & UART_UARTMIS_TXMIS_BITS) {
      // clear interrupt
      hw_write_masked(&uart_get_hw(UART_NUM)->icr, 1 << UART_UARTICR_TXIC_LSB,
                      UART_UARTICR_BEIC_BITS);
    }
    ClearFifo();
  }
  portYIELD_FROM_ISR(woken);
}

static void UartInit() {
  uart_init(UART_NUM, 9600);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(UART_NUM, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_NUM, true);
  uart_set_hw_flow(UART_NUM, false, false);

  int UART_IRQ = UART_NUM == uart0 ? UART0_IRQ : UART1_IRQ;
  irq_set_exclusive_handler(UART_IRQ, UartEvent);
  uart_set_irq_enables(UART_NUM, true, true);
}

// data must remain valid throughout the command lifecycle!
// returns actual response length; if error occurred, returns negative value
static int
BatteryCall(const BatteryRequest* req) {
  if (xSemaphoreTake(batteryMutex, req->timeout) == pdFALSE) {
    return -1;
  }

  int len;

  ClearFifo();

  currentRequest = req;
  comm = REQ_FRAME_HEAD;
  waitingTask = xTaskGetCurrentTaskHandle();
  int UART_IRQ = UART_NUM == uart0 ? UART0_IRQ : UART1_IRQ;
  irq_set_enabled(UART_IRQ, true);

  // Kickstart
  UartEvent();

  uint32_t val;
  if (xTaskNotifyWait(0, 0xFFFFFFFF, &val, req->timeout) == pdFALSE) {
    len = -1;
  } else {
    if (val == kMagicOk) {
      len = actualResponseLength;
    } else {
      len = -2;
    }
  }
  irq_set_enabled(UART_IRQ, false);

  currentRequest = nullptr;
  comm = IDLE;
  waitingTask = nullptr;

  xSemaphoreGive(batteryMutex);
  return len;
}

uint8_t ttt[100];
void ControllerTask(void *_) {
  UartInit();
  batteryMutex = xSemaphoreCreateMutex();
  while (true) {
    BatteryRequest req = {
        .address = kBatteryAddress,
        .command = 0x1,
        .req_len = 0,
        .resp = ttt,
        .resp_len = sizeof(ttt),
        .timeout = 200,
    };
    volatile int aaa = BatteryCall(&req);
    vTaskDelay(100);
  }
}