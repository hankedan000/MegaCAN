#include <EEPROM.h>
#include "EndianUtils.h"
#include "FlashUtils.h"
#include <logging.h>
#include "MegaCAN_ExtDevice.h"
#include "MegaCAN_RT_BroadcastHelper.h"
#include "tables.h"

#include <TaskScheduler.h>
// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
// #define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding

// Required for MegaCAN library
SET_MEGA_CAN_SIG("OpenGPIO");
SET_MEGA_CAN_REV("OpenGPIO-0.1.0     ");

// CAN related variables
#define CAN_CS  10
#define CAN_INT 2
#define CAN_ID  1
#define CAN_MSG_BUFFER_SIZE 40

#define RT_BCAST_OFFSET PAGE2_FIELD_OFFSET(rtBcast)

MegaCAN::CAN_Msg can_buff[CAN_MSG_BUFFER_SIZE];
MegaCAN::ExtDevice gpio(CAN_CS,CAN_ID,CAN_INT,can_buff,CAN_MSG_BUFFER_SIZE,TABLES,NUM_TABLES);

// Scheduler
Scheduler ts;

#define ADC_READY_MASK 0x1000
volatile uint16_t adcBuff[6];

void can_isr();

void
send_rt_bcast_group(
  uint16_t baseId,
  uint8_t group)
{
  uint16_t id = baseId + group;
  DEBUG("sending rt group %d; id %d",group,id);
  
  switch (group)
  {
    case 0:// 00: ADC0,ADC1,ADC2,ADC3
      gpio.send11bitFrame(id,8,((uint8_t*)(&outPC) + OUTPC_FIELD_OFFSET(adc0)));
      break;
    case 1:// 01: ADC4,ADC5
      gpio.send11bitFrame(id,4,((uint8_t*)(&outPC) + OUTPC_FIELD_OFFSET(adc4)));
      break;
    default:
      WARN("bad rt group %d",group);
      break;
  }
}

void
setup()
{
  Serial.begin(115200);
  setupLogging();

  cli();

  // MCP2515 configuration
  gpio.init();
  pinMode(CAN_INT, INPUT_PULLUP);// Configuring pin for CAN interrupt input
  attachInterrupt(digitalPinToInterrupt(CAN_INT), can_isr, LOW);

  // setup real-time broadcast class
  MegaCAN::RT_Bcast.setup(&ts, RT_BCAST_OFFSET, send_rt_bcast_group);

  // Setup analog inputs
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  // setup ADC sampling
  ADMUX = 0 | bit(REFS0); // sample ADC0 first
  // enable ADC, and request ADC complete interrupt
  ADCSRA = bit(ADEN) | bit(ADIF) | bit(ADIE);
  ADCSRA |= 0x7;// ADC prescaler 128 (Arduino default)

  // enabled interrupts
  sei();
  
  INFO("setup complete!");

  // kick off first ADC conversion
  ADCSRA |= bit(ADSC);
}

void
loop()
{
  uint32_t loopStartTimeUs = micros();
  gpio.handle();
  ts.execute();

  // perform ADC mapping operations on readings that are 'ready'
  for (uint8_t i=0; i<6; i++)
  {
    if (adcBuff[i] & ADC_READY_MASK)
    {
      applyNewADC(adcBuff[i] & ~ADC_READY_MASK,i);
      adcBuff[i] &= ~ADC_READY_MASK;// clear 'ready' bit
    }
  }

  // perform real-time data transmission
  MegaCAN::RT_Bcast.execute();

  // update status0
  outPC.status0.bits.needsBurn = gpio.needsBurn();
  outPC.status0.bits.flashDataLost = gpio.flashDataLost();
  outPC.status0.bits.currFlashTable = gpio.currFlashTable();

  // update CAN status registers
  outPC.canStatus = gpio.getCAN_Status();
  outPC.canErrorCount = gpio.getLogicErrorCount();

  // update loop time register
  uint16_t loopTimeUs = micros() - loopStartTimeUs;
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}

void
applyNewADC(
  uint16_t adc,
  uint8_t adcIdx)
{
  uint16_t *outPC_ADC_Values = &outPC.adc0;
  uint8_t mappingCtrlVal = EEPROM.read(PAGE1_FIELD_OFFSET(adc0MappingCtrl) + adcIdx);
  ADC_MappingControl_T *mappingCtrl = (ADC_MappingControl_T *)(&mappingCtrlVal);

  if (mappingCtrl->bits.enabled)
  {
    // linear interpolate the ADC value based on the select mapping curve
    int16_t mapVal;
    switch (mappingCtrl->bits.curve)
    {
      case 0:
        mapVal = FlashUtils::lerpS16(
          PAGE1_FIELD_OFFSET(adcMappingCurveA_xBins),
          PAGE1_FIELD_OFFSET(adcMappingCurveA_yBins),
          ADC_MAPPING_CURVE_N_BINS,
          adc);
        break;
      case 1:
        mapVal = FlashUtils::lerpS16(
          PAGE1_FIELD_OFFSET(adcMappingCurveB_xBins),
          PAGE1_FIELD_OFFSET(adcMappingCurveB_yBins),
          ADC_MAPPING_CURVE_N_BINS,
          adc);
        break;
      case 2:
        mapVal = FlashUtils::lerpS16(
          PAGE1_FIELD_OFFSET(adcMappingCurveC_xBins),
          PAGE1_FIELD_OFFSET(adcMappingCurveC_yBins),
          ADC_MAPPING_CURVE_N_BINS,
          adc);
        break;
      case 3:
        mapVal = FlashUtils::lerpS16(
          PAGE1_FIELD_OFFSET(adcMappingCurveD_xBins),
          PAGE1_FIELD_OFFSET(adcMappingCurveD_yBins),
          ADC_MAPPING_CURVE_N_BINS,
          adc);
        break;
      default:
        mapVal = adc;
        break;
    }
    
    // store mapped value in outPC
    EndianUtils::setBE(outPC_ADC_Values[adcIdx], (uint16_t)mapVal);
  }
  else
  {
    // no mapping, just assign RAW value
    EndianUtils::setBE(outPC_ADC_Values[adcIdx], adc);
  }
}

// external interrupt service routine for CAN message on MCP2515
void can_isr()
{
  gpio.interrupt();
}

// ADC complete ISR
ISR (ADC_vect)
{
  uint8_t currADC = ADMUX & 0x7;

  // store ADC in buffer and mark 'ready' so main can perform mapping on it.
  // we don't want to perform any EEPROM accesses inside the ISR since that can
  // cause data race conditions on the AVR's EEPROM access registers.
  adcBuff[currADC] = ADC_READY_MASK | ADC;
  
  // setup sampling of next ADC pin (ADC 0 to 5)
  uint8_t nextADC = (currADC == 5 ? 0 : currADC + 1);
  ADMUX = bit(REFS0) | nextADC;
  // start ADC conversion
  ADCSRA |= bit(ADSC);
}