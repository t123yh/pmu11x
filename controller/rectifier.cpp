//
// Created by 田韵豪 on 2023/1/31.
//

#include <FreeRTOS.h>
#include <queue.h>
#include "rectifier.h"
#include "../can/can2040.h"
#include "../rtt/SEGGER_RTT.h"
#include <pico/platform.h>
#include <hardware/irq.h>
#include <hardware/clocks.h>
#include <cstring>

static const uint tx_pin = 23;
static const uint rx_pin = 22;

static QueueHandle_t dataQueue;
bool rectifierRunningStatus, rectifierActiveStatus;

struct __attribute__((packed)) psuMessageId {
  uint8_t count : 1;
  uint8_t rev : 6;
  uint8_t fromSrc : 1;
  uint8_t cmdId : 8;
  uint8_t addr : 7;
  uint8_t protoId : 6;
  uint8_t :3;
};

struct psuData {
  uint16_t regB;
  uint8_t :8;
  uint8_t control;
  uint32_t valB;
};

struct psuMessage {
  union {
    uint32_t id;
    psuMessageId structId;
  };

  union {
    uint8_t data[8];
    psuData structData;
  };
};

static BaseType_t woken;
static void __no_inline_not_in_flash_func(can2040Callback)
(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  if (notify == CAN2040_NOTIFY_RX) {
    psuMessage recv{};
    recv.id = msg->id & 0x1FFFFFFF;
    memcpy(recv.data, msg->data, 8);
    /*
    if (recv.structId.cmdId != 0x40)
      SEGGER_RTT_printf(0, "ID: %08X, addr: %02X, cmdId: %02X, DATA: %08X%08X\n", recv.id, recv.structId.addr, recv.structId.cmdId, __builtin_bswap32(msg->data32[0]), __builtin_bswap32(msg->data32[1]));
      */
    if (recv.structId.cmdId != 0x11) {
      xQueueSendFromISR(dataQueue, &recv, &woken);
    } else {
      if (recv.id == 0x1001117E) {
        rectifierActiveStatus = !recv.data[3];
        SEGGER_RTT_printf(0, "Rect have alarm %d\n", rectifierActiveStatus);
      } else if (recv.id == 0x108111FE) {
        rectifierRunningStatus = !!recv.data[5];
        SEGGER_RTT_printf(0, "Rect running %d\n", rectifierRunningStatus);
      }
    }
  }
}

static can2040 cd;

static void PIOx_IRQHandler() {
  woken = pdFALSE;
  can2040_pio_irq_handler(&cd);
  portYIELD_FROM_ISR(woken);
}

void RectifierInit() {
  dataQueue = xQueueCreate(15, sizeof(psuMessage));
  can2040_setup(&cd, 0);
  can2040_callback_config(&cd, can2040Callback);
  // Enable irqs
  irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
  irq_set_priority(PIO0_IRQ_0, 1);
  irq_set_enabled(PIO0_IRQ_0, true);
  can2040_start(&cd, clock_get_hz(clk_sys), 125000, rx_pin, tx_pin);
}

static bool transmitMessage(const psuMessage & msg) {
  can2040_msg m{};
  m.id = msg.id | CAN2040_ID_EFF;
  m.dlc = 8;
  memcpy(m.data, msg.data, 8);
  return can2040_transmit(&cd, &m) == 0;
}

const static TickType_t timeout = 200;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-narrowing-conversions"
bool RectifierQuery(RectifierInfo* info) {
  psuMessage msg{};
  msg.id = 0x108140FE;
  transmitMessage(msg);
  TickType_t start = xTaskGetTickCount();
  TickType_t end = start + timeout;
  do {
    int waitTime = end - xTaskGetTickCount();
    if (waitTime > 0) {
      if (!xQueueReceive(dataQueue, &msg, waitTime)) {
        return false;
      }
      if (msg.structId.cmdId == 0x40) {
        uint32_t val = __builtin_bswap32(msg.structData.valB);
        switch (__builtin_bswap16(msg.structData.regB)) {
          case 0x10e:
            info->status_value = msg.structData.regB;
            break;
          case 0x170:
            info->input_power = val / 1024.0;
            break;
          case 0x171:
            info->input_frequency = val / 1024.0;
            break;
          case 0x172:
            info->input_current = val / 1024.0;
            break;
          case 0x173:
            info->output_power = val / 1024.0;
            break;
          case 0x174:
            info->efficiency = val / 1024.0;
            break;
          case 0x175:
            info->output_voltage = val / 1024.0;
            break;
          case 0x176:
            info->max_output_current = val / 30.0;
            break;
          case 0x178:
            info->input_voltage = val / 1024.0;
            break;
          case 0x17F:
            info->output_module_temp = val / 1024.0;
            break;
          case 0x180:
            info->input_module_temp = val / 1024.0;
            break;
          case 0x182:
            info->output_current = val / 1024.0;
            break;
          case 0x183:
            info->alarm_value = msg.structData.valB;
            return true;
            break;
        }
      }
    } else {
      break;
    }
  } while (true);
  return false;
}
#pragma clang diagnostic pop

bool RectifierControl(uint8_t cmdId, uint16_t reg, uint8_t val1, uint32_t val2) {
  psuMessage msg{}, recvMsg;
  msg.id = 0x108100FE;
  msg.structId.cmdId = cmdId;
  msg.structData.regB = __builtin_bswap16(reg);
  msg.structData.control = val1;
  msg.structData.valB = __builtin_bswap32(val2);
  transmitMessage(msg);
  TickType_t start = xTaskGetTickCount();
  TickType_t end = start + timeout;
  do {
    int waitTime = end - xTaskGetTickCount();
    if (waitTime > 0) {
      if (!xQueueReceive(dataQueue, &recvMsg, waitTime)) {
        return false;
      }
      if (msg.structId.cmdId == cmdId) {
        if (memcmp(msg.data, recvMsg.data, 8) == 0) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      break;
    }
  } while (true);
  return false;
}

bool RectifierPowerControl(bool on) {
  return RectifierControl(0x80, 0x132, on ? 0 : 1, 0);
}

bool RectifierOfflineVoltageControl(float voltage) {
  if (voltage > 58.5) {
    voltage = 58.5;
  }
  if (voltage < 48.2) {
    voltage = 48.2;
  }
  return RectifierControl(0x80, 0x101, 0, voltage * 1024);
}
bool RectifierOnlineVoltageControl(float voltage) {
  if (voltage > 58.5) {
    voltage = 58.5;
  }
  if (voltage < 42) {
    voltage = 42;
  }
  return RectifierControl(0x80, 0x100, 0, voltage * 1024);
}
