#pragma once

#include <stdint.h>

#define bswap16(x) __builtin_bswap16(x)
#define bswap32(x) __builtin_bswap32(x)

// standard message types
#define MSG_CMD     0
#define MSG_REQ     1
#define MSG_RSP     2
#define MSG_XSUB    3
#define MSG_BURN    4
#define OUTMSG_REQ  5
#define OUTMSG_RSP  6
#define MSG_XTND    7
// extended message types
#define MSG_FWD     8
#define MSG_CRC     9
#define MSG_REQX    12
#define MSG_BURNACK 14
#define MSG_PROT    0x80
#define MSG_WCR     0x81
#define MSG_SPND    0x82

#define TABLE_NO_REV 14
#define TABLE_NO_SIG 15

#define MAX_REVISION_BYTES 60
#define MAX_SIGNATURE_BYTES 20

// defines for accessing MSG_PROT data field
#define GET_MSG_PROT_MYVARBLK(data8_ptr)    ((data8_ptr)[1])
#define GET_MSG_PROT_MYVAROFFSET(data8_ptr) ((uint16_t)((data8_ptr)[3] >> 5) | ((uint16_t)((data8_ptr)[2]) << 3))
#define GET_MSG_PROT_VARBYT(data8_ptr)      ((data8_ptr)[3] & 0xf)

struct MS_HDR_t{
  uint8_t          : 2;
  uint8_t   tableH : 1;
  uint8_t   tableL : 4;
  uint8_t   toId   : 4;
  uint8_t   fromId : 4;
  uint8_t   type   : 3;
  uint16_t  offset : 11;
  uint8_t          : 3;

  uint32_t
  marshal()
  {
    return *reinterpret_cast<uint32_t*>(this);
  }
}__attribute__((packed));

inline uint8_t
getTable(
		const MS_HDR_t *hdr)
{
	return (hdr->tableH << 4) | hdr->tableL;
}

inline void
setTable(
		MS_HDR_t *hdr,
		uint8_t table)
{
	hdr->tableL = table & 0xF;
	hdr->tableH = (table >> 4) & 0x1;
}

struct MSG_REQ_t{
  uint8_t   rspTable   : 5;
  uint8_t              : 3;
  uint8_t   rspOffsetH : 8;
  uint8_t   rspLength  : 4;
  uint8_t              : 1;
  uint8_t   rspOffsetL : 3;
}__attribute__((packed));

inline uint16_t
getOffset(
		const MSG_REQ_t *req)
{
	return req->rspOffsetH << 3 | req->rspOffsetL;
}

#define MSG_GET_U16(buff, offset) (bswap16(*(const uint16_t *)(buff + offset)))
#define MSG_GET_U32(buff, offset) (bswap32(*(const uint32_t *)(buff + offset)))

namespace MegaCAN
{

  // struct to help access the fixed points attributes of the megasquirt's
  // realtime broadcast messages
  template<typename VAL_T, uint16_t MULT, uint32_t DIV>
  struct MsgAttr
  {
    using val_t = VAL_T;

    VAL_T value;// raw value

    MsgAttr(VAL_T v)
    : value(v)
    {}

    uint16_t mult() {return MULT;}
    uint16_t div() {return DIV;}

    VAL_T whole() {return (value * MULT) / DIV;}
    VAL_T frac() {return (value * MULT) % DIV;}
    float flt() {return ((float)(value) * MULT) / DIV;}
  };
  
  struct RtMsg00_t
  {
    uint8_t data[8];

    // Seconds ECU has been on
    // units: s
    uint16_t seconds() {return MSG_GET_U16(data,0);}

    // Main pulsewidth bank 1
    // units: ms
    MsgAttr<uint16_t,1,1000> pw1() const {return MSG_GET_U16(data,2);}

    // Main pulsewidth bank 2
    // units: ms
    MsgAttr<uint16_t,1,1000> pw2() const {return MSG_GET_U16(data,4);}

    // Engine RPM
    // units: RPM
    uint16_t rpm() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg01_t
  {
    uint8_t data[8];

    // Final ignition spark advance
    // units: deg BTDC
    MsgAttr<int16_t,1,10> adv_deg() const {return MSG_GET_U16(data,0);}

    // Bitfield of batch fire injector events
    uint8_t squirt() {return data[2];}

    // Bitfield of engine status
    uint8_t engine() {return data[3];}

    // Bank 1 AFR target
    // units: AFR
    MsgAttr<uint8_t,1,10> afrtgt1() const {return data[4];}

    // Bank 2 AFR targer
    // units: AFR
    MsgAttr<uint8_t,1,10> afrtgt2() const {return data[5];}

    // not used
    uint8_t wbo2_en1() {return data[6];}

    // not used
    uint8_t wbo2_en2() {return data[7];}
  };

  struct RtMsg02_t
  {
    uint8_t data[8];

    // Barometric pressure
    // units: kPa
    MsgAttr<int16_t,1,10> baro() const {return MSG_GET_U16(data,0);}

    // Manifold air pressure
    // units: kPa
    MsgAttr<int16_t,1,10> map() const {return MSG_GET_U16(data,2);}

    // Manifold air temperature
    // units: deg F
    MsgAttr<int16_t,1,10> mat() const {return MSG_GET_U16(data,4);}

    // Coolant temperature
    // units: deg F
    MsgAttr<int16_t,1,10> clt() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg03_t
  {
    uint8_t data[8];

    // Throttle position
    // units: %
    MsgAttr<int16_t,1,10> tps() const {return MSG_GET_U16(data,0);}

    // Battery voltage
    // units: V
    MsgAttr<int16_t,1,10> batt() const {return MSG_GET_U16(data,2);}

    // AFR1 (Deprecated on MS3)
    // units: AFR
    MsgAttr<int16_t,1,10> afr1_old() const {return MSG_GET_U16(data,4);}

    // AFR2 (Deprecated on MS3)
    // units: AFR
    MsgAttr<int16_t,1,10> afr2_old() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg04_t
  {
    uint8_t data[8];

    // Indication of knock input
    // units: %
    MsgAttr<int16_t,1,10> knock() const {return MSG_GET_U16(data,0);}

    // EGO bank 1 correction
    // units: %
    MsgAttr<int16_t,1,10> egocor1() const {return MSG_GET_U16(data,2);}

    // EGO bank2 correction
    // units: %
    MsgAttr<int16_t,1,10> egocor2() const {return MSG_GET_U16(data,4);}

    // Air density correction
    // units: %
    MsgAttr<int16_t,1,10> aircor() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg05_t
  {
    uint8_t data[8];

    // Warmup correction
    // units: %
    MsgAttr<int16_t,1,10> warmcor() const {return MSG_GET_U16(data,0);}

    // TPS-based acceleration
    // units: %
    MsgAttr<int16_t,1,10> tpsaccel() const {return MSG_GET_U16(data,2);}

    // TPS-based fuel cut
    // units: %
    MsgAttr<int16_t,1,10> tpsfuelcut() const {return MSG_GET_U16(data,4);}

    // Barometric fuel correction
    // units: %
    MsgAttr<int16_t,1,10> barocor() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg06_t
  {
    uint8_t data[8];

    // Total fuel correction
    // units: %
    MsgAttr<int16_t,1,10> totalcor() const {return MSG_GET_U16(data,0);}

    // VE value table/bank 1
    // units: %
    MsgAttr<int16_t,1,10> ve1() const {return MSG_GET_U16(data,2);}

    // VE value table/bank 2
    // units: %
    MsgAttr<int16_t,1,10> ve2() const {return MSG_GET_U16(data,4);}

    // Stepper idle step number
    // units: step
    int16_t iacstep() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg07_t
  {
    uint8_t data[8];

    // Cold advance
    // units: deg
    MsgAttr<int16_t,1,10> cold_adv_deg() const {return MSG_GET_U16(data,0);}

    // Rate of change of TPS
    // units: %/s
    MsgAttr<int16_t,1,10> TPSdot() const {return MSG_GET_U16(data,2);}

    // Rate of change of MAP
    // units: kPa/s
    int16_t MAPdot() {return MSG_GET_U16(data,4);}

    // Rate of change of RPM
    // units: RPM/s
    MsgAttr<int16_t,10,1> RPMdot() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg08_t
  {
    uint8_t data[8];

    // Synthetic 'load' from MAF
    // units: %
    MsgAttr<int16_t,1,10> MAFload() const {return MSG_GET_U16(data,0);}

    // 'Load' used for fuel table lookup e.g. equals
    // units: %
    MsgAttr<int16_t,1,10> fuelload() const {return MSG_GET_U16(data,2);}

    // Adjustment to fuel from Flex
    // units: %
    MsgAttr<int16_t,1,10> fuelcor() const {return MSG_GET_U16(data,4);}

    // Mass Air Flow
    // units: g/s
    MsgAttr<int16_t,1,100> MAF() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg09_t
  {
    uint8_t data[8];

    // Voltage from O2#1 (Deprecated on MS3)
    // units: V
    MsgAttr<int16_t,1,100> egoV1() const {return MSG_GET_U16(data,0);}

    // Voltage from O2#2 (Deprecated on MS3)
    // units: V
    MsgAttr<int16_t,1,100> egoV2() const {return MSG_GET_U16(data,2);}

    // Main ignition dwell
    // units: ms
    MsgAttr<uint16_t,1,10> dwell() const {return MSG_GET_U16(data,4);}

    // Trailing ignition dwell
    // units: ms
    MsgAttr<uint16_t,1,10> dwell_trl() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg10_t
  {
    uint8_t data[8];

    // ECU status bitfield
    uint8_t status1() {return data[0];}

    // ECU status bitfield
    uint8_t status2() {return data[1];}

    // ECU status bitfield
    uint8_t status3() {return data[2];}

    // Not typically used
    uint8_t status4() {return data[3];}

    // Not typically used
    int16_t status5() {return MSG_GET_U16(data,4);}

    // ECU status bitfield
    uint8_t status6() {return data[6];}

    // ECU status bitfield
    uint8_t status7() {return data[7];}
  };

  struct RtMsg11_t
  {
    uint8_t data[8];

    // 'Load' used for fuelling on modified table
    // units: %
    MsgAttr<int16_t,1,10> fuelload2() const {return MSG_GET_U16(data,0);}

    // 'Load' used for ignition table lookup
    // units: %
    MsgAttr<int16_t,1,10> ignload() const {return MSG_GET_U16(data,2);}

    // 'Load' used for modifier ignition table lookup
    // units: %
    MsgAttr<int16_t,1,10> ignload2() const {return MSG_GET_U16(data,4);}

    // Estimated intake air temperature
    // units: deg F
    MsgAttr<int16_t,1,10> airtemp() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg12_t
  {
    uint8_t data[8];

    // Calculated volume of fuel on intake walls from
    // units: us
    MsgAttr<int32_t,1,100> wallfuel1() const {return MSG_GET_U32(data,0);}

    // Second channel of same
    // units: us
    MsgAttr<int32_t,1,100> wallfuel2() const {return MSG_GET_U32(data,4);}
  };

  struct RtMsg13_t
  {
    uint8_t data[8];

    // Generic sensor input 1 (gpioadc0 on MS2)
    MsgAttr<int16_t,1,10> sensors1() const {return MSG_GET_U16(data,0);}

    // Generic sensor input 2 (gpioadc1 on MS2)
    MsgAttr<int16_t,1,10> sensors2() const {return MSG_GET_U16(data,2);}

    // Generic sensor input 3 (gpioadc2 on MS2)
    MsgAttr<int16_t,1,10> sensors3() const {return MSG_GET_U16(data,4);}

    // Generic sensor input 4 (gpioadc3 on MS2)
    MsgAttr<int16_t,1,10> sensors4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg14_t
  {
    uint8_t data[8];

    // Generic sensor input 5 (gpioadc4 on MS2)
    MsgAttr<int16_t,1,10> sensors5() const {return MSG_GET_U16(data,0);}

    // Generic sensor input 6 (gpioadc5 on MS2)
    MsgAttr<int16_t,1,10> sensors6() const {return MSG_GET_U16(data,2);}

    // Generic sensor input 7 (gpioadc6 on MS2)
    MsgAttr<int16_t,1,10> sensors7() const {return MSG_GET_U16(data,4);}

    // Generic sensor input 8 (gpioadc7 on MS2)
    MsgAttr<int16_t,1,10> sensors8() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg15_t
  {
    uint8_t data[8];

    // Generic sensor input 9 (adc6 on MS2)
    MsgAttr<int16_t,1,10> sensors9() const {return MSG_GET_U16(data,0);}

    // Generic sensor input 10 (adc7 on MS2)
    MsgAttr<int16_t,1,10> sensors10() const {return MSG_GET_U16(data,2);}

    // Generic sensor input 11
    MsgAttr<int16_t,1,10> sensors11() const {return MSG_GET_U16(data,4);}

    // Generic sensor input 12
    MsgAttr<int16_t,1,10> sensors12() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg16_t
  {
    uint8_t data[8];

    // Generic sensor input 13
    MsgAttr<int16_t,1,10> sensors13() const {return MSG_GET_U16(data,0);}

    // Generic sensor input 14
    MsgAttr<int16_t,1,10> sensors14() const {return MSG_GET_U16(data,2);}

    // Generic sensor input 15
    MsgAttr<int16_t,1,10> sensors15() const {return MSG_GET_U16(data,4);}

    // Generic sensor input 16
    MsgAttr<int16_t,1,10> sensors16() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg17_t
  {
    uint8_t data[8];

    // Target boost - channel 1
    // units: kPa
    MsgAttr<int16_t,1,10> boost_targ_1() const {return MSG_GET_U16(data,0);}

    // Target boost - channel 2
    // units: kPa
    MsgAttr<int16_t,1,10> boost_targ_2() const {return MSG_GET_U16(data,2);}

    // Duty cycle on boost solenoid 1
    // units: %
    uint8_t boostduty() {return data[4];}

    // Duty cycle on boost solenoid 2
    // units: %
    uint8_t boostduty2() {return data[5];}

    // MAF voltage (synthesised for frequency
    // units: V
    MsgAttr<int16_t,1,1000> maf_volts() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg18_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#1
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq1() const {return MSG_GET_U16(data,0);}

    // Sequential Pulsewidth for cyl#2
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq2() const {return MSG_GET_U16(data,2);}

    // Sequential Pulsewidth for cyl#3
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq3() const {return MSG_GET_U16(data,4);}

    // Sequential Pulsewidth for cyl#4
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg19_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#5
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq5() const {return MSG_GET_U16(data,0);}

    // Sequential Pulsewidth for cyl#6
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq6() const {return MSG_GET_U16(data,2);}

    // Sequential Pulsewidth for cyl#7
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq7() const {return MSG_GET_U16(data,4);}

    // Sequential Pulsewidth for cyl#8
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq8() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg20_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#9
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq9() const {return MSG_GET_U16(data,0);}

    // Sequential Pulsewidth for cyl#10
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq10() const {return MSG_GET_U16(data,2);}

    // Sequential Pulsewidth for cyl#11
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq11() const {return MSG_GET_U16(data,4);}

    // Sequential Pulsewidth for cyl#12
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq12() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg21_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#13
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq13() const {return MSG_GET_U16(data,0);}

    // Sequential Pulsewidth for cyl#14
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq14() const {return MSG_GET_U16(data,2);}

    // Sequential Pulsewidth for cyl#15
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq15() const {return MSG_GET_U16(data,4);}

    // Sequential Pulsewidth for cyl#16
    // units: ms
    MsgAttr<int16_t,1,1000> pwseq16() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg22_t
  {
    uint8_t data[8];

    // EGT 1
    // units: deg F
    MsgAttr<int16_t,1,10> egt1() const {return MSG_GET_U16(data,0);}

    // EGT 2
    // units: deg F
    MsgAttr<int16_t,1,10> egt2() const {return MSG_GET_U16(data,2);}

    // EGT 3
    // units: deg F
    MsgAttr<int16_t,1,10> egt3() const {return MSG_GET_U16(data,4);}

    // EGT 4
    // units: deg F
    MsgAttr<int16_t,1,10> egt4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg23_t
  {
    uint8_t data[8];

    // EGT 5
    // units: deg F
    MsgAttr<int16_t,1,10> egt5() const {return MSG_GET_U16(data,0);}

    // EGT 6
    // units: deg F
    MsgAttr<int16_t,1,10> egt6() const {return MSG_GET_U16(data,2);}

    // EGT 7
    // units: deg F
    MsgAttr<int16_t,1,10> egt7() const {return MSG_GET_U16(data,4);}

    // EGT 8
    // units: deg F
    MsgAttr<int16_t,1,10> egt8() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg24_t
  {
    uint8_t data[8];

    // EGT 9
    // units: deg F
    MsgAttr<int16_t,1,10> egt9() const {return MSG_GET_U16(data,0);}

    // EGT 10
    // units: deg F
    MsgAttr<int16_t,1,10> egt10() const {return MSG_GET_U16(data,2);}

    // EGT 11
    // units: deg F
    MsgAttr<int16_t,1,10> egt11() const {return MSG_GET_U16(data,4);}

    // EGT 12
    // units: deg F
    MsgAttr<int16_t,1,10> egt12() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg25_t
  {
    uint8_t data[8];

    // EGT 13
    // units: deg F
    MsgAttr<int16_t,1,10> egt13() const {return MSG_GET_U16(data,0);}

    // EGT 14
    // units: deg F
    MsgAttr<int16_t,1,10> egt14() const {return MSG_GET_U16(data,2);}

    // EGT 15
    // units: deg F
    MsgAttr<int16_t,1,10> egt15() const {return MSG_GET_U16(data,4);}

    // EGT 16
    // units: deg F
    MsgAttr<int16_t,1,10> egt16() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg26_t
  {
    uint8_t data[8];

    // Duty cycle to nitrous solenoid 1
    // units: %
    uint8_t nitrous1_duty() {return data[0];}

    // Duty cycle to nitrous solenoid 2
    // units: %
    uint8_t nitrous2_duty() {return data[1];}

    // Timer used internally for nitrous system
    // units: s
    MsgAttr<uint16_t,1,1000> nitrous_timer_ou() const {return MSG_GET_U16(data,2);}

    // Fuel pulsewidth added due to nitrous system
    // units: ms
    MsgAttr<int16_t,1,1000> n2o_addfuel() const {return MSG_GET_U16(data,4);}

    // Timing retard due to nitrous system
    // units: deg
    MsgAttr<int16_t,1,10> n2o_retard() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg27_t
  {
    uint8_t data[8];

    // PWM period 1 from remote board
    int16_t canpwmin1() {return MSG_GET_U16(data,0);}

    // PWM period 2 from remote board
    int16_t canpwmin2() {return MSG_GET_U16(data,2);}

    // PWM period 3 from remote board
    int16_t canpwmin3() {return MSG_GET_U16(data,4);}

    // PWM period 4 from remote board
    int16_t canpwmin4() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg28_t
  {
    uint8_t data[8];

    // Closed-loop idle target RPM
    // units: RPM
    uint16_t cl_idle_targ_rpm() {return MSG_GET_U16(data,0);}

    // ADC count from TPS
    int16_t tpsadc() {return MSG_GET_U16(data,2);}

    // 'Load' used for EAE calc
    // units: %
    MsgAttr<int16_t,1,10> eaeload() const {return MSG_GET_U16(data,4);}

    // 'Load' used for AFR table lookups
    // units: %
    MsgAttr<int16_t,1,10> afrload() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg29_t
  {
    uint8_t data[8];

    // Fuel correction from EAE - channel 1
    // units: %
    MsgAttr<uint16_t,1,10> EAEfcor1() const {return MSG_GET_U16(data,0);}

    // Fuel correction from EAE - channel 2
    // units: %
    MsgAttr<uint16_t,1,10> EAEfcor2() const {return MSG_GET_U16(data,2);}

    // Rate of change of VSS1
    // units: ms-2
    MsgAttr<int16_t,1,10> VSS1dot() const {return MSG_GET_U16(data,4);}

    // Rate of change of VSS2
    // units: ms-2
    MsgAttr<int16_t,1,10> VSS2dot() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg30_t
  {
    uint8_t data[8];

    // External accelerometer X
    // units: ms-2
    MsgAttr<int16_t,1,1000> accelx() const {return MSG_GET_U16(data,0);}

    // External accelerometer Y
    // units: ms-2
    MsgAttr<int16_t,1,1000> accely() const {return MSG_GET_U16(data,2);}

    // External accelerometer Z
    // units: ms-2
    MsgAttr<int16_t,1,1000> accelz() const {return MSG_GET_U16(data,4);}

    // Volume level on audio input
    uint8_t stream_level() {return data[6];}

    // Duty cycle to water injection solenoid
    // units: %
    uint8_t water_duty() {return data[7];}
  };

  struct RtMsg31_t
  {
    uint8_t data[8];

    // AFR cyl#1
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR1() const {return data[0];}

    // AFR cyl#2
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR2() const {return data[1];}

    // AFR cyl#3
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR3() const {return data[2];}

    // AFR cyl#4
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR4() const {return data[3];}

    // AFR cyl#5
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR5() const {return data[4];}

    // AFR cyl#6
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR6() const {return data[5];}

    // AFR cyl#7
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR7() const {return data[6];}

    // AFR cyl#8
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR8() const {return data[7];}
  };

  struct RtMsg32_t
  {
    uint8_t data[8];

    // AFR cyl#9
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR9() const {return data[0];}

    // AFR cyl#10
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR10() const {return data[1];}

    // AFR cyl#11
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR11() const {return data[2];}

    // AFR cyl#12
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR12() const {return data[3];}

    // AFR cyl#13
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR13() const {return data[4];}

    // AFR cyl#14
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR14() const {return data[5];}

    // AFR cyl#15
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR15() const {return data[6];}

    // AFR cyl#16
    // units: AFR
    MsgAttr<uint8_t,1,10> AFR16() const {return data[7];}
  };

  struct RtMsg33_t
  {
    uint8_t data[8];

    // Generic PWM duty 1
    // units: %
    uint8_t duty_pwm1() {return data[0];}

    // Generic PWM duty 2
    // units: %
    uint8_t duty_pwm2() {return data[1];}

    // Generic PWM duty 3
    // units: %
    uint8_t duty_pwm3() {return data[2];}

    // Generic PWM duty 4
    // units: %
    uint8_t duty_pwm4() {return data[3];}

    // Generic PWM duty 5
    // units: %
    uint8_t duty_pwm5() {return data[4];}

    // Generic PWM duty 6
    // units: %
    uint8_t duty_pwm6() {return data[5];}

    // Current gear selected
    int8_t gear() {return data[6];}

    // Engine status bitfield
    uint8_t status8() {return data[7];}
  };

  struct RtMsg34_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#1
    // units: V
    MsgAttr<int16_t,489,100000> EGOv1() const {return MSG_GET_U16(data,0);}

    // Voltage from O2 cyl#2
    // units: V
    MsgAttr<int16_t,489,100000> EGOv2() const {return MSG_GET_U16(data,2);}

    // Voltage from O2 cyl#3
    // units: V
    MsgAttr<int16_t,489,100000> EGOv3() const {return MSG_GET_U16(data,4);}

    // Voltage from O2 cyl#4
    // units: V
    MsgAttr<int16_t,489,100000> EGOv4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg35_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#5
    // units: V
    MsgAttr<int16_t,489,100000> EGOv5() const {return MSG_GET_U16(data,0);}

    // Voltage from O2 cyl#6
    // units: V
    MsgAttr<int16_t,489,100000> EGOv6() const {return MSG_GET_U16(data,2);}

    // Voltage from O2 cyl#7
    // units: V
    MsgAttr<int16_t,489,100000> EGOv7() const {return MSG_GET_U16(data,4);}

    // Voltage from O2 cyl#8
    // units: V
    MsgAttr<int16_t,489,100000> EGOv8() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg36_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#9
    // units: V
    MsgAttr<int16_t,489,100000> EGOv9() const {return MSG_GET_U16(data,0);}

    // Voltage from O2 cyl#10
    // units: V
    MsgAttr<int16_t,489,100000> EGOv10() const {return MSG_GET_U16(data,2);}

    // Voltage from O2 cyl#11
    // units: V
    MsgAttr<int16_t,489,100000> EGOv11() const {return MSG_GET_U16(data,4);}

    // Voltage from O2 cyl#12
    // units: V
    MsgAttr<int16_t,489,100000> EGOv12() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg37_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#13
    // units: V
    MsgAttr<int16_t,489,100000> EGOv13() const {return MSG_GET_U16(data,0);}

    // Voltage from O2 cyl#14
    // units: V
    MsgAttr<int16_t,489,100000> EGOv14() const {return MSG_GET_U16(data,2);}

    // Voltage from O2 cyl#15
    // units: V
    MsgAttr<int16_t,489,100000> EGOv15() const {return MSG_GET_U16(data,4);}

    // Voltage from O2 cyl#16
    // units: V
    MsgAttr<int16_t,489,100000> EGOv16() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg38_t
  {
    uint8_t data[8];

    // EGO correction cyl#1
    // units: %
    MsgAttr<int16_t,1,10> EGOcor1() const {return MSG_GET_U16(data,0);}

    // EGO correction cyl#2
    // units: %
    MsgAttr<int16_t,1,10> EGOcor2() const {return MSG_GET_U16(data,2);}

    // EGO correction cyl#3
    // units: %
    MsgAttr<int16_t,1,10> EGOcor3() const {return MSG_GET_U16(data,4);}

    // EGO correction cyl#4
    // units: %
    MsgAttr<int16_t,1,10> EGOcor4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg39_t
  {
    uint8_t data[8];

    // EGO correction cyl#5
    // units: %
    MsgAttr<int16_t,1,10> EGOcor5() const {return MSG_GET_U16(data,0);}

    // EGO correction cyl#6
    // units: %
    MsgAttr<int16_t,1,10> EGOcor6() const {return MSG_GET_U16(data,2);}

    // EGO correction cyl#7
    // units: %
    MsgAttr<int16_t,1,10> EGOcor7() const {return MSG_GET_U16(data,4);}

    // EGO correction cyl#8
    // units: %
    MsgAttr<int16_t,1,10> EGOcor8() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg40_t
  {
    uint8_t data[8];

    // EGO correction cyl#9
    // units: %
    MsgAttr<int16_t,1,10> EGOcor9() const {return MSG_GET_U16(data,0);}

    // EGO correction cyl#10
    // units: %
    MsgAttr<int16_t,1,10> EGOcor10() const {return MSG_GET_U16(data,2);}

    // EGO correction cyl#11
    // units: %
    MsgAttr<int16_t,1,10> EGOcor11() const {return MSG_GET_U16(data,4);}

    // EGO correction cyl#12
    // units: %
    MsgAttr<int16_t,1,10> EGOcor12() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg41_t
  {
    uint8_t data[8];

    // EGO correction cyl#13
    // units: %
    MsgAttr<int16_t,1,10> EGOcor13() const {return MSG_GET_U16(data,0);}

    // EGO correction cyl#14
    // units: %
    MsgAttr<int16_t,1,10> EGOcor14() const {return MSG_GET_U16(data,2);}

    // EGO correction cyl#15
    // units: %
    MsgAttr<int16_t,1,10> EGOcor15() const {return MSG_GET_U16(data,4);}

    // EGO correction cyl#16
    // units: %
    MsgAttr<int16_t,1,10> EGOcor16() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg42_t
  {
    uint8_t data[8];

    // Vehicle Speed 1
    // units: ms-1
    MsgAttr<uint16_t,1,10> VSS1() const {return MSG_GET_U16(data,0);}

    // Vehicle Speed 2
    // units: ms-1
    MsgAttr<uint16_t,1,10> VSS2() const {return MSG_GET_U16(data,2);}

    // Vehicle Speed 3
    // units: ms-1
    MsgAttr<uint16_t,1,10> VSS3() const {return MSG_GET_U16(data,4);}

    // Vehicle Speed 4
    // units: ms-1
    MsgAttr<uint16_t,1,10> VSS4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg43_t
  {
    uint8_t data[8];

    // Sync-loss counter
    uint8_t synccnt() {return data[0];}

    // Sync-loss reason code
    uint8_t syncreason() {return data[1];}

    // SDcard file number
    uint16_t sd_filenum() {return MSG_GET_U16(data,2);}

    // SDcard error number
    uint8_t sd_error() {return data[4];}

    // SDcard internal code
    uint8_t sd_phase() {return data[5];}

    // SDcard status bitfield
    uint8_t sd_status() {return data[6];}

    // Calculated error in ignition timing
    // units: %
    int8_t timing_err() {return data[7];}
  };

  struct RtMsg44_t
  {
    uint8_t data[8];

    // VVT actual angle 1
    // units: deg
    MsgAttr<int16_t,1,10> vvt_ang1() const {return MSG_GET_U16(data,0);}

    // VVT actual angle 2
    // units: deg
    MsgAttr<int16_t,1,10> vvt_ang2() const {return MSG_GET_U16(data,2);}

    // VVT actual angle 3
    // units: deg
    MsgAttr<int16_t,1,10> vvt_ang3() const {return MSG_GET_U16(data,4);}

    // VVT actual angle 4
    // units: deg
    MsgAttr<int16_t,1,10> vvt_ang4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg45_t
  {
    uint8_t data[8];

    // VVT target angle 1
    // units: deg
    MsgAttr<int16_t,1,10> vvt_target1() const {return MSG_GET_U16(data,0);}

    // VVT target angle 2
    // units: deg
    MsgAttr<int16_t,1,10> vvt_target2() const {return MSG_GET_U16(data,2);}

    // VVT target angle 3
    // units: deg
    MsgAttr<int16_t,1,10> vvt_target3() const {return MSG_GET_U16(data,4);}

    // VVT target angle 4
    // units: deg
    MsgAttr<int16_t,1,10> vvt_target4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg46_t
  {
    uint8_t data[8];

    // VVT solenoid 1 duty cycle
    // units: %
    MsgAttr<uint8_t,392,1000> vvt_duty1() const {return data[0];}

    // VVT solenoid 2 duty cycle
    // units: %
    MsgAttr<uint8_t,392,1000> vvt_duty2() const {return data[1];}

    // VVT solenoid 3 duty cycle
    // units: %
    MsgAttr<uint8_t,392,1000> vvt_duty3() const {return data[2];}

    // VVT solenoid 4 duty cycle
    // units: %
    MsgAttr<uint8_t,392,1000> vvt_duty4() const {return data[3];}

    // Injection Timing Angle (primary)
    // units: deg BTDC
    MsgAttr<int16_t,1,10> inj_timing_pri() const {return MSG_GET_U16(data,4);}

    // Injection Timing Angle (secondary)
    // units: deg BTDC
    MsgAttr<int16_t,1,10> inj_timing_sec() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg47_t
  {
    uint8_t data[8];

    // Ethanol content of fuel from Flex sensor
    // units: %
    MsgAttr<int16_t,1,10> fuel_pct() const {return MSG_GET_U16(data,0);}

    // TPSdot based accel
    // units: %
    MsgAttr<int16_t,1,10> tps_accel() const {return MSG_GET_U16(data,2);}

    // Shaft speed 1
    // units: RPM
    MsgAttr<uint16_t,10,1> SS1() const {return MSG_GET_U16(data,4);}

    // Shaft speed 2
    // units: RPM
    MsgAttr<uint16_t,10,1> SS2() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg48_t
  {
    uint8_t data[8];

    // Knock % cyl #1
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl1() const {return data[0];}

    // Knock % cyl #2
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl2() const {return data[1];}

    // Knock % cyl #3
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl3() const {return data[2];}

    // Knock % cyl #4
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl4() const {return data[3];}

    // Knock % cyl #5
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl5() const {return data[4];}

    // Knock % cyl #6
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl6() const {return data[5];}

    // Knock % cyl #7
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl7() const {return data[6];}

    // Knock % cyl #8
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl8() const {return data[7];}
  };

  struct RtMsg49_t
  {
    uint8_t data[8];

    // Knock % cyl #9
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl9() const {return data[0];}

    // Knock % cyl #10
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl10() const {return data[1];}

    // Knock % cyl #11
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl11() const {return data[2];}

    // Knock % cyl #12
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl12() const {return data[3];}

    // Knock % cyl #13
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl13() const {return data[4];}

    // Knock % cyl #14
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl14() const {return data[5];}

    // Knock % cyl #15
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl15() const {return data[6];}

    // Knock % cyl #16
    // units: %
    MsgAttr<uint8_t,4,10> knock_cyl16() const {return data[7];}
  };

  struct RtMsg50_t
  {
    uint8_t data[8];

    // MAPdot based accel
    // units: %
    MsgAttr<int16_t,1,10> map_accel() const {return MSG_GET_U16(data,0);}

    // Total accel
    // units: %
    MsgAttr<int16_t,1,10> total_accel() const {return MSG_GET_U16(data,2);}

    // Timer for timed-launch retard
    // units: s
    MsgAttr<uint16_t,1,1000> launch_timer() const {return MSG_GET_U16(data,5);}

    // Launch retard
    // units: deg
    MsgAttr<int16_t,1,10> launch_retard() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg51_t
  {
    uint8_t data[8];

    // CPU portA bitfield
    uint8_t porta() {return data[0];}

    // CPU portB bitfield
    uint8_t portb() {return data[1];}

    // CPU portE/portH bitfield
    uint8_t porteh() {return data[2];}

    // CPU portK bitfield
    uint8_t portk() {return data[3];}

    // CPU portM/portJ bitfield
    uint8_t portmj() {return data[4];}

    // CPU portP bitfield
    uint8_t portp() {return data[5];}

    // CPU portT bitfield
    uint8_t portt() {return data[6];}

    // CEL error code
    uint8_t cel_errorcode() {return data[7];}
  };

  struct RtMsg52_t
  {
    uint8_t data[8];

    // CAN input 1 bitfield (CAN port 1 on MS2)
    uint8_t canin1() {return data[0];}

    // CAN input 2 bitfield (CAN port 2 on MS2)
    uint8_t canin2() {return data[1];}

    // CAN output 1 bitfield (CAN port 3 on MS2)
    uint8_t canout() {return data[2];}

    // Knock retard
    // units: deg
    MsgAttr<uint8_t,1,10> knk_rtd() const {return data[3];}

    // Average fuel flow
    // units: cc/min
    uint16_t fuelflow() {return MSG_GET_U16(data,4);}

    // Average fuel consumption
    // units: l/km
    uint16_t fuelcons() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg53_t
  {
    uint8_t data[8];

    // Fuel pressure 1
    // units: kPa
    MsgAttr<int16_t,1,10> fuel_press1() const {return MSG_GET_U16(data,0);}

    // Fuel pressure 2
    // units: kPa
    MsgAttr<int16_t,1,10> fuel_press2() const {return MSG_GET_U16(data,2);}

    // Fuel temperature 1
    // units: deg F
    MsgAttr<int16_t,1,10> fuel_temp1() const {return MSG_GET_U16(data,4);}

    // Fuel temperature 2
    // units: deg F
    MsgAttr<int16_t,1,10> fuel_temp2() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg54_t
  {
    uint8_t data[8];

    // Battery current (alternator system)
    // units: A
    MsgAttr<int16_t,1,10> batt_cur() const {return MSG_GET_U16(data,0);}

    // CEL status bitfield
    uint16_t cel_status() {return MSG_GET_U16(data,2);}

    // Fuel pump output duty
    // units: %
    MsgAttr<uint8_t,392,1000> fp_duty() const {return data[4];}

    // Alternator field output duty
    // units: %
    uint8_t alt_duty() {return data[5];}

    // Alternator measured load-sense duty
    // units: %
    uint8_t load_duty() {return data[6];}

    // Alternator target voltage
    // units: V
    MsgAttr<uint8_t,1,10> alt_targv() const {return data[7];}
  };

  struct RtMsg55_t
  {
    uint8_t data[8];

    // Main code loop execution time
    // units: us
    uint16_t looptime() {return MSG_GET_U16(data,0);}

    // Fuel temperature correction
    // units: %
    MsgAttr<uint16_t,1,10> fueltemp_cor() const {return MSG_GET_U16(data,2);}

    // Fuel pressure correction
    // units: %
    MsgAttr<uint16_t,1,10> fuelpress_cor() const {return MSG_GET_U16(data,4);}

    // Long term trim correction
    // units: %
    MsgAttr<int8_t,1,10> ltt_cor() const {return data[6];}

    // Unused
    // units: 
    uint8_t sp1() {return data[7];}
  };

  struct RtMsg56_t
  {
    uint8_t data[8];

    // Traction control retard
    // units: deg
    MsgAttr<int16_t,1,10> tc_retard() const {return MSG_GET_U16(data,0);}

    // CEL retard
    // units: deg
    MsgAttr<int16_t,1,10> cel_retard() const {return MSG_GET_U16(data,2);}

    // Fuel-cut (overrun) retard
    // units: deg
    MsgAttr<int16_t,1,10> fc_retard() const {return MSG_GET_U16(data,4);}

    // ALS added fuel
    // units: ms
    MsgAttr<int16_t,1,1000> als_addfuel() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg57_t
  {
    uint8_t data[8];

    // Base timing from tables
    // units: deg
    MsgAttr<int16_t,1,10> base_advance() const {return MSG_GET_U16(data,0);}

    // Idle correction advance
    // units: deg
    MsgAttr<int16_t,1,10> idle_cor_advance() const {return MSG_GET_U16(data,2);}

    // MAT retard
    // units: deg
    MsgAttr<int16_t,1,10> mat_retard() const {return MSG_GET_U16(data,4);}

    // Flex advance
    // units: deg
    MsgAttr<int16_t,1,10> flex_advance() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg58_t
  {
    uint8_t data[8];

    // Timing lookup from table 1
    // units: deg
    MsgAttr<int16_t,1,10> adv1() const {return MSG_GET_U16(data,0);}

    // Timing lookup from table 2
    // units: deg
    MsgAttr<int16_t,1,10> adv2() const {return MSG_GET_U16(data,2);}

    // Timing lookup from table 3
    // units: deg
    MsgAttr<int16_t,1,10> adv3() const {return MSG_GET_U16(data,4);}

    // Timing lookup from table 4
    // units: deg
    MsgAttr<int16_t,1,10> adv4() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg59_t
  {
    uint8_t data[8];

    // Revlimiter 'soft' retard
    // units: deg
    MsgAttr<int16_t,1,10> revlim_retard() const {return MSG_GET_U16(data,0);}

    // ALS timing change
    // units: deg
    MsgAttr<int16_t,1,10> als_timing() const {return MSG_GET_U16(data,2);}

    // External advance (e.g. trans)
    // units: deg
    MsgAttr<int16_t,1,10> ext_advance() const {return MSG_GET_U16(data,4);}

    // Injector deadtime in use (#1)
    // units: ms
    MsgAttr<int16_t,1,1000> deadtime1() const {return MSG_GET_U16(data,6);}
  };

  struct RtMsg60_t
  {
    uint8_t data[8];

    // Launch control timing
    // units: deg
    MsgAttr<int16_t,1,10> launch_timing() const {return MSG_GET_U16(data,0);}

    // 3-step timing
    // units: deg
    MsgAttr<int16_t,1,10> step3_timing() const {return MSG_GET_U16(data,2);}

    // Wheel-speed based launch retard
    // units: deg
    MsgAttr<int16_t,1,10> vsslaunch_retard() const {return MSG_GET_U16(data,4);}

    // CEL status 2
    uint16_t cel_status2() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg61_t
  {
    uint8_t data[8];

    // External GPS latitude deg
    // units: deg
    int8_t gps_latdeg() {return data[0];}

    // GPS latitude minutes
    // units: min
    uint8_t gps_latmin() {return data[1];}

    // GPS latitude milli-minutes
    // units: mmin
    uint16_t gps_latmmin() {return MSG_GET_U16(data,2);}

    // GPS longitude degree
    // units: deg
    uint8_t gps_londeg() {return data[4];}

    // GPS longitude minute
    // units: min
    uint8_t gps_lonmin() {return data[5];}

    // GPS longitude milli-minutes
    // units: mmin
    uint16_t gps_lonmmin() {return MSG_GET_U16(data,6);}
  };

  struct RtMsg62_t
  {
    uint8_t data[8];

    // GPS status byte (bit 0 = E/W)
    uint8_t gps_outstatus() {return data[0];}

    // GPS altitude km
    // units: km
    int8_t gps_altk() {return data[1];}

    // GPS altitude m
    // units: m
    uint16_t gps_altm() {return MSG_GET_U16(data,2);}

    // GPS speed
    // units: ms-1
    MsgAttr<uint16_t,1,10> gps_speed() const {return MSG_GET_U16(data,4);}

    // GPS course
    // units: deg
    MsgAttr<uint16_t,1,10> gps_course() const {return MSG_GET_U16(data,6);}
  };

}
