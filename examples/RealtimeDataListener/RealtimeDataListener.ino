#include <EEPROM.h>
#include <EndianUtils.h>
#include <FlashUtils.h>
#include <logging_impl_lite.h>

#include "RealtimeDataListener.h"

// CAN related variables
#define CAN_CS  10
#define CAN_INT 2
#define CAN_ID  1
#define CAN_MSG_BUFFER_SIZE 1// only 1 since we handle messages immediately

MegaCAN::CAN_Msg canBuff[CAN_MSG_BUFFER_SIZE];
RealtimeDataListener rtdl(CAN_CS,CAN_ID,CAN_INT,canBuff,CAN_MSG_BUFFER_SIZE);

void canISR();

void
setup()
{
  setupLogging(115200);

  cli();

  // MCP2515 configuration
  rtdl.init();
  pinMode(CAN_INT, INPUT_PULLUP);// Configuring pin for CAN interrupt input
  attachInterrupt(digitalPinToInterrupt(CAN_INT), canISR, LOW);

  // enabled interrupts
  sei();
  
  INFO("setup complete!");
}

void
loop()
{
  INFO("rpm = %d", rtdl.data().m0.rpm());
  INFO("clt = %d", rtdl.data().m2.clt());
}

// external interrupt service routine for CAN message on MCP2515
void canISR()
{
  rtdl.interrupt();
}
