#include "interboard_comm.hpp"

#include "can.h"
#include "can_driver.h"

namespace {

constexpr uint32_t kInterboardRetreatCmdId = 0x168U;
constexpr uint8_t kMagic0 = 0xA5U;
constexpr uint8_t kMagic1 = 0x5AU;

void SendOnBus(CAN_HandleTypeDef* hcan, bool enable) {
  CAN_TxHeaderTypeDef tx_header = {
      .StdId = kInterboardRetreatCmdId,
      .ExtId = 0U,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
      .DLC = 8U,
      .TransmitGlobalTime = DISABLE,
  };

  uint8_t data[8] = {0};
  data[0] = kMagic0;
  data[1] = kMagic1;
  data[2] = enable ? 1U : 0U;

  (void)CAN_SendMessage(hcan, &tx_header, data);
}

} // namespace

void InterboardComm_Init(void) {}

void InterboardComm_SendRetreatCommand(bool enable) {
  SendOnBus(&hcan1, enable);
  SendOnBus(&hcan2, enable);
}

bool InterboardComm_IsRetreatRequested(void) { return false; }
