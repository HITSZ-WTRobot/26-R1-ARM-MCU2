#include "interboard_comm.hpp"

#include "usart.h"

#include <string.h>

namespace {

constexpr uint8_t kFrameHeader = 0xAAU;
constexpr uint8_t kCmdRetreat = 0x01U;
constexpr uint8_t kCmdTarget = 0x02U;
constexpr uint8_t kAckMask = 0x80U;
constexpr uint8_t kTlvTypeX = 0x01U;
constexpr uint8_t kTlvTypeY = 0x02U;
constexpr uint8_t kTlvTypeYaw = 0x03U;
constexpr uint8_t kTlvI32Len = 4U;

constexpr uint8_t kMaxPayloadLen = 32U; // CMD(1) + DATA
constexpr uint32_t kTxTimeoutMs = 5U;
constexpr uint32_t kAckTimeoutMs = 20U;

volatile uint8_t g_rx_ack_cmd = 0U;

uint8_t g_rx_state = 0U;
uint8_t g_rx_payload_len = 0U;
uint8_t g_rx_payload_fill = 0U;
uint8_t g_rx_checksum = 0U;
uint8_t g_rx_payload[kMaxPayloadLen] = {0};

static uint8_t CalcChecksum(uint8_t len, const uint8_t *payload) {
  uint8_t sum = len;
  if (payload) {
    for (uint8_t i = 0; i < len; ++i) {
      sum ^= payload[i];
    }
  }
  return sum;
}

static uint8_t AppendTlvI32(uint8_t *out_buf,
                            uint8_t offset,
                            uint8_t type,
                            int32_t value) {
  if (!out_buf) {
    return offset;
  }
  out_buf[offset++] = type;
  out_buf[offset++] = kTlvI32Len;
  out_buf[offset++] = (uint8_t)(value & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 16) & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 24) & 0xFF);
  return offset;
}

static void SendFrame(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
  const uint8_t payload_len = (uint8_t)(1U + data_len);
  uint8_t frame[2U + kMaxPayloadLen + 1U] = {0};
  uint8_t payload[kMaxPayloadLen] = {0};

  if (payload_len > kMaxPayloadLen) {
    return;
  }

  payload[0] = cmd;
  if (data && data_len > 0U) {
    memcpy(&payload[1], data, data_len);
  }

  frame[0] = kFrameHeader;
  frame[1] = payload_len;
  memcpy(&frame[2], payload, payload_len);
  frame[2U + payload_len] = CalcChecksum(payload_len, payload);

  (void)HAL_UART_Transmit(&huart4,
                          frame,
                          (uint16_t)(3U + payload_len),
                          kTxTimeoutMs);
}

static bool WaitAck(uint8_t cmd, uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start) <= timeout_ms) {
    if (g_rx_ack_cmd == cmd) {
      g_rx_ack_cmd = 0U;
      return true;
    }
  }
  return false;
}

static void HandleFrame(uint8_t payload_len, const uint8_t *payload) {
  if (!payload || payload_len < 1U) {
    return;
  }

  const uint8_t cmd = payload[0];
  if ((cmd & kAckMask) != 0U) {
    g_rx_ack_cmd = (uint8_t)(cmd & (uint8_t)~kAckMask);
  }
}

static void SendCommandWithAck(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
  g_rx_ack_cmd = 0U;
  SendFrame(cmd, data, data_len);
  (void)WaitAck(cmd, kAckTimeoutMs);
}

} // namespace

void InterboardComm_Init(void) {
  g_rx_state = 0U;
  g_rx_payload_len = 0U;
  g_rx_payload_fill = 0U;
  g_rx_ack_cmd = 0U;
}

void InterboardComm_OnUartByte(uint8_t byte) {
  switch (g_rx_state) {
  case 0U:
    g_rx_state = (byte == kFrameHeader) ? 1U : 0U;
    break;
  case 1U:
    if (byte == 0U || byte > kMaxPayloadLen) {
      g_rx_state = 0U;
    } else {
      g_rx_payload_len = byte;
      g_rx_payload_fill = 0U;
      g_rx_state = 2U;
    }
    break;
  case 2U:
    g_rx_payload[g_rx_payload_fill++] = byte;
    if (g_rx_payload_fill >= g_rx_payload_len) {
      g_rx_state = 3U;
    }
    break;
  case 3U:
    g_rx_checksum = byte;
    if (g_rx_checksum == CalcChecksum(g_rx_payload_len, g_rx_payload)) {
      HandleFrame(g_rx_payload_len, g_rx_payload);
    }
    g_rx_state = 0U;
    break;
  default:
    g_rx_state = 0U;
    break;
  }
}

void InterboardComm_SendRetreatCommand(bool enable) {
  const uint8_t payload = (uint8_t)(enable ? 1U : 0U);
  SendCommandWithAck(kCmdRetreat, &payload, 1U);
}

bool InterboardComm_IsRetreatRequested(void) { return false; }

void InterboardComm_SendTargetMm(int32_t x_mm, int32_t y_mm, int32_t yaw_mm) {
  uint8_t data[18] = {0};
  uint8_t len = 0U;
  len = AppendTlvI32(data, len, kTlvTypeX, x_mm);
  len = AppendTlvI32(data, len, kTlvTypeY, y_mm);
  len = AppendTlvI32(data, len, kTlvTypeYaw, yaw_mm);
  SendCommandWithAck(kCmdTarget, data, len);
}
