#pragma once

#include <mcp_can/mcp_can.h>
#include <MegaCAN/hal/CAN_Bus.h>

namespace MegaCAN
{

  class MCP2515_CAN_Bus : public HAL::CAN_Bus
  {
  public:
    MCP2515_CAN_Bus() = delete;

    explicit
    MCP2515_CAN_Bus(
      const uint8_t csPin);
    
    HAL::CAN_Bus::RetCode
    readAny(
      HAL::CAN_Msg & msg) final;
    
    HAL::CAN_Bus::RetCode
    sendAny(
      const HAL::CAN_Id & id,
      const uint8_t * data,
      const uint8_t len,
      const bool waitForSend = 1u) final;

    inline MCP_CAN & mcpCAN() {return can_;}

  private:
    MCP_CAN can_;

  };

}
