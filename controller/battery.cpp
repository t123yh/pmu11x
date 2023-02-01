//
// Created by 田韵豪 on 2023/1/23.
//

#include <cstdint>
#include "battery.h"
#include "../led.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <pico.h>
#include <hardware/uart.h>
#include <cstring>
#include <algorithm>
#include <hardware/irq.h>
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

#define ARRAY_SIZE(a)                               \
  ((sizeof(a) / sizeof(*(a))) /                     \
  static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

const size_t kMaxCells = ARRAY_SIZE(BatteryInfo::cells);
const size_t kMaxTempSensors = ARRAY_SIZE(BatteryInfo::temperatures);
const size_t kMaxAlarms = ARRAY_SIZE(BatteryInfo::alarm.bytes);

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

  int len = -1;
  const int max_retries = 3;
  int retries = max_retries;

  while (retries-- && len < 0) {
    ClearFifo();
    if (retries == max_retries - 1)
      vTaskDelay(20);

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
  }

  currentRequest = nullptr;
  comm = IDLE;
  waitingTask = nullptr;

  xSemaphoreGive(batteryMutex);
  return len;
}

struct ValueParser {
  const uint8_t* data;
  uint16_t len;
  uint16_t pos;

  ValueParser(const uint8_t* data, uint16_t len) : data(data), len(len), pos(0) { }

  uint8_t currentField() const {
    if (eof())
      return 0;
    return data[pos];
  }

  bool eof() const {
    return pos >= len;
  }

  static void copy(void* dest, const void* src, size_t len) {
    const uint8_t* start = ((const uint8_t*) src) + len - 1;
    uint8_t* destu = (uint8_t*)dest;
    for (int i = 0; i < len; i++) {
      *destu = *start;
      destu++;
      start--;
    }
  }

  template <typename T> T readSingle() {
    if (pos + 2 > len) {
      pos = len;
      return T{};
    }

    T ret;
    pos++;
    size_t fieldLen = data[pos] * 2;
    pos++;
    copy(&ret, data + pos, std::min(sizeof(T), fieldLen));
    pos += fieldLen;
    return ret;
  }

  template <typename T> uint8_t readMultiple(T* fields, uint8_t max) {
    if (pos + 2 >= len) {
      pos = len;
      return 0;
    }
    pos++;
    size_t count = data[pos];
    pos++;
    int copied = 0;
    for (int i = 0; i < count; i++) {
      if (pos + sizeof(T) > len) {
        pos = len;
        return copied;
      }
      if (i < max) {
        copied++;
        copy(fields + i, data + pos, sizeof(T));
      }
      pos += sizeof(T);
    }
    return copied;
  }

  size_t readArray(uint8_t* buf, size_t bufLen) {
    if (pos + 2 >= len) {
      pos = len;
      return 0;
    }
    pos++;
    size_t fieldLen = data[pos] * 2;
    pos++;
    if (pos + fieldLen > len) {
      return 0;
    }

    size_t copied = std::min(fieldLen, bufLen);
    memcpy(buf, data + pos, copied);
    pos += fieldLen;

    return copied;
  }

  void skip() {
    if (pos + 2 > len) {
      pos = len;
    }

    pos++;
    size_t fieldLen = data[pos] * 2;
    pos++;
    pos += fieldLen;
  }
};

void parseValue(BatteryInfo* result, uint8_t* ptr, uint8_t len) {
  BatteryInfo & info = *result;
  ValueParser parser(ptr, len);
  while (!parser.eof()) {
    uint8_t f = parser.currentField();
    switch (f) {
      case 1: // battery cell info
        struct __attribute__ ((__packed__)) {
          uint16_t value:13;
          bool underVoltage:1;
          bool overVoltage:1;
          bool balance:1;
        } raw[kMaxCells];
        info.cells_count = parser.readMultiple(raw, kMaxCells);
        for (int i = 0; i < info.cells_count; i++) {
          info.cells[i].value = raw[i].value / 1000.0;
          info.cells[i].over_voltage = raw[i].overVoltage;
          info.cells[i].under_voltage = raw[i].underVoltage;
          info.cells[i].balance = raw[i].balance;
        }
        break;
      case 2:
        info.current = (((float)parser.readSingle<uint16_t>()) - 30000.0) / 100.0;
        break;
      case 3:
        info.soc = parser.readSingle<uint16_t>() / 100.0;
        break;
      case 4:
        info.full_capacity = parser.readSingle<uint16_t>() / 100.0;
        break;
      case 5:
        struct __attribute__ ((__packed__)) {
          uint8_t value;
          uint8_t status;
        } temp_raw[kMaxTempSensors];
        info.temperatures_count = parser.readMultiple(temp_raw, kMaxTempSensors);
        for (int i = 0; i < info.temperatures_count; i++) {
          info.temperatures[i] = temp_raw[i].value - 50;
        }
        break;
      case 6:
        info.alarm.size = parser.readArray(info.alarm.bytes, kMaxAlarms);
        break;
      case 7:
        info.loop = parser.readSingle<uint16_t>();
        break;
      case 8:
        info.voltage_sum = parser.readSingle<uint16_t>() / 100.0;
        break;
      case 9:
        info.soh = parser.readSingle<uint16_t>() / 100.0;
        break;
      default:
        parser.skip();
        break;
    }
  }
}

void BatteryInit() {
  UartInit();
  batteryMutex = xSemaphoreCreateMutex();
}

bool GetBatteryInfo(BatteryInfo* info) {
  size_t batRetSize = 100;
  uint8_t* buf = static_cast<uint8_t *>(pvPortMalloc(batRetSize));
  BatteryRequest req = {
      .address = kBatteryAddress,
      .command = 0x1,
      .req_len = 0,
      .resp = buf,
      .resp_len = batRetSize,
      .timeout = 200,
  };
  int ret = BatteryCall(&req);
  if (ret > 0) {
    parseValue(info, buf, (uint8_t)ret);
  }
  vPortFree(buf);
  return ret > 0;
}

bool SendBatteryCommand(const BatteryCommand& cmd) {
  BatteryRequest req = {
      .address = kBatteryAddress,
      .command = cmd.cmd,
      .req = &cmd.val,
      .req_len = 1,
      .resp = nullptr,
      .resp_len = 0,
      .timeout = 100,
  };
  int ret = BatteryCall(&req);
  return ret >= 0;
}