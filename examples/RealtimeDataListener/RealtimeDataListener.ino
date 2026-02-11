#define MC_LOG_ENABLED

#include <EEPROM.h>
#include <EndianUtils.h>
#include <FlashUtils.h>
#include <MegaCAN/Logging.h>

#include "RealtimeDataListener.h"

// Required for MegaCAN library
DECL_MEGA_CAN_REV("OpenGPIO");
DECL_MEGA_CAN_SIG("OpenGPIO-0.1.0     ");

// CAN related variables
#define CAN_CS  10
#define CAN_INT 2
#define CAN_ID  0// don't care since we're only listening
#define CAN_MSG_BUFFER_SIZE 1// only 1 because we handle 11bit frames immediately

MegaCAN::CAN_Msg canBuff[CAN_MSG_BUFFER_SIZE];
RealtimeDataListener rtdl(CAN_CS,CAN_ID,CAN_INT,canBuff,CAN_MSG_BUFFER_SIZE);

void canISR();

#ifdef MC_LOG_ENABLED
void
mcLogCallback(
  const MegaCAN::LogLevel lvl,
  const char * msg)
{
  switch (lvl)
  {
    case MegaCAN::LogLevel::DEBUG:
      Serial.print(F("DEBUG | "));
      break;
    case MegaCAN::LogLevel::INFO:
      Serial.print(F("INFO  | "));
      break;
    case MegaCAN::LogLevel::WARN:
      Serial.print(F("WARN  | "));
      break;
    case MegaCAN::LogLevel::ERROR:
      Serial.print(F("ERROR | "));
      break;
    default:
      return;
  }
  Serial.println(msg);
}

void setupLogging()
{
  Serial.begin(115200);
  MegaCAN::Logging.setCallback(mcLogCallback);
}
#else
void setupLogging()
{
  // do nothing
}
#endif

void
setup()
{
  setupLogging();

  cli();

  // MCP2515 configuration
  rtdl.init();
  pinMode(CAN_INT, INPUT_PULLUP);// Configuring pin for CAN interrupt input
  attachInterrupt(digitalPinToInterrupt(CAN_INT), canISR, LOW);

  // enabled interrupts
  sei();
  
  MC_LOG_INFO("setup complete!");
}

uint16_t lastDisplaySeconds = 0;

void
loop()
{
  rtdl.handle();

  // print engine variables every second
  const uint16_t ecuSeconds = rtdl.data().m0.seconds();
  if (lastDisplaySeconds != ecuSeconds)
  {
    MC_LOG_INFO("=========================================================");
    MC_LOG_INFO(
      "msg00: seconds %d, pw1 %d, pw2 %d, rpm %d",
      ecuSeconds,
      rtdl.data().m0.pw1().whole(),
      rtdl.data().m0.pw2().whole(),
      rtdl.data().m0.rpm());
    MC_LOG_INFO(
      "msg01: adv_deg %d, squirt %d, engine %02x, afrtgt1 %d, afrtgt2 %d, wbo2_en1 %d, wbo2_en2 %d",
      rtdl.data().m1.adv_deg().whole(),
      rtdl.data().m1.squirt(),
      rtdl.data().m1.engine(),
      rtdl.data().m1.afrtgt1(),
      rtdl.data().m1.afrtgt2(),
      rtdl.data().m1.wbo2_en1(),
      rtdl.data().m1.wbo2_en2());
    MC_LOG_INFO(
      "msg02: baro %d, map %d, mat %d, clt %d",
      rtdl.data().m2.baro().whole(),
      rtdl.data().m2.map().whole(),
      rtdl.data().m2.mat().whole(),
      rtdl.data().m2.clt().whole());
    MC_LOG_INFO(
      "msg03: tps %d, batt %d, afr1_old %d, afr2_old %d",
      rtdl.data().m3.tps().whole(),
      rtdl.data().m3.batt().whole(),
      rtdl.data().m3.afr1_old().whole(),
      rtdl.data().m3.afr2_old().whole());
    MC_LOG_INFO(
      "msg10: status1 %02x, status2 %02x, status3 %02x, status4 %02x, status5 %04x, status6 %02x, status7 %02x",
      rtdl.data().m10.status1(),
      rtdl.data().m10.status2(),
      rtdl.data().m10.status3(),
      rtdl.data().m10.status4(),
      rtdl.data().m10.status5(),
      rtdl.data().m10.status6(),
      rtdl.data().m10.status7());
    lastDisplaySeconds = ecuSeconds;
  }
}

// external interrupt service routine for CAN message on MCP2515
void canISR()
{
  rtdl.interrupt();
}
