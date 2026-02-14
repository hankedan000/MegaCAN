#include <MegaCAN/hal_impl/arduino/MCP2515_CAN_Bus.h>

namespace MegaCAN
{

  MCP2515_CAN_Bus::MCP2515_CAN_Bus(
    const uint8_t csPin)
  : can_(csPin)
  {}

  HAL::CAN_Bus::RetCode
  MCP2515_CAN_Bus::readAny(
    HAL::CAN_Msg & msg)
  {
    uint32_t idOut = 0u;
    uint8_t extOut = 0u;
    uint8_t lenOut = 0u;
    const auto rc = can_.readMsgBuf(&idOut, &extOut, &lenOut, msg.data.data());
    switch (rc)
    {
      case CAN_NOMSG:
        return HAL::CAN_Bus::RetCode::NO_MSG;
      case CAN_OK:
        // data was directly read into the buff, but we need to
        // resize it to match the length that was read.
        msg.data.resize_nofill(lenOut);
        return HAL::CAN_Bus::RetCode::OK;
    }
    MC_PANIC("MCP2515_CAN_Bus::readAny - unexpected retcode");
  }

  HAL::CAN_Bus::RetCode
  MCP2515_CAN_Bus::sendAny(
    const HAL::CAN_Id & id,
    const uint8_t * data,
    const uint8_t len,
    const bool waitForSend)
  {
    const auto rc = can_.sendMsgBuf(id.getId(), id.isExt(), len, data, waitForSend);
    switch (rc)
    {
      case CAN_GETTXBFTIMEOUT:
        return HAL::CAN_Bus::RetCode::BUFFER_BUSY;
      case CAN_SENDMSGTIMEOUT:
        return HAL::CAN_Bus::RetCode::TIMEOUT;
      case CAN_OK:
        return HAL::CAN_Bus::RetCode::OK;
    }
    MC_PANIC("MCP2515_CAN_Bus::sendAny - unexpected retcode");
  }

}