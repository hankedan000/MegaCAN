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
    uint16_t seconds_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t seconds_whole() const {return MSG_GET_U16(data,0) * 1 / 1;}
    uint16_t seconds_frac() const  {return MSG_GET_U16(data,0) * 1 / 1;}
    float    seconds_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1;}

    // Main pulsewidth bank 1
    // units: ms
    uint16_t pw1_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t pw1_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t pw1_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    pw1_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Main pulsewidth bank 2
    // units: ms
    uint16_t pw2_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t pw2_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t pw2_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    pw2_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Engine RPM
    // units: RPM
    uint16_t rpm_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t rpm_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t rpm_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    rpm_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg01_t
  {
    uint8_t data[8];

    // Final ignition spark advance
    // units: deg BTDC
    uint16_t adv_deg_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t adv_deg_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t adv_deg_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    adv_deg_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Bitfield of batch fire injector events
    uint16_t squirt_raw() const   {return data[2];}
    uint16_t squirt_whole() const {return data[2] * 1 / 1;}
    uint16_t squirt_frac() const  {return data[2] * 1 / 1;}
    float    squirt_flt() const   {return (float)(data[2]) * 1 / 1;}

    // Bitfield of engine status
    uint16_t engine_raw() const   {return data[3];}
    uint16_t engine_whole() const {return data[3] * 1 / 1;}
    uint16_t engine_frac() const  {return data[3] * 1 / 1;}
    float    engine_flt() const   {return (float)(data[3]) * 1 / 1;}

    // Bank 1 AFR target
    // units: AFR
    uint16_t afrtgt1_raw() const   {return data[4];}
    uint16_t afrtgt1_whole() const {return data[4] * 1 / 10;}
    uint16_t afrtgt1_frac() const  {return data[4] * 1 / 10;}
    float    afrtgt1_flt() const   {return (float)(data[4]) * 1 / 10;}

    // Bank 2 AFR targer
    // units: AFR
    uint16_t afrtgt2_raw() const   {return data[5];}
    uint16_t afrtgt2_whole() const {return data[5] * 1 / 10;}
    uint16_t afrtgt2_frac() const  {return data[5] * 1 / 10;}
    float    afrtgt2_flt() const   {return (float)(data[5]) * 1 / 10;}

    // not used
    uint16_t wbo2_en1_raw() const   {return data[6];}
    uint16_t wbo2_en1_whole() const {return data[6] * 1 / 1;}
    uint16_t wbo2_en1_frac() const  {return data[6] * 1 / 1;}
    float    wbo2_en1_flt() const   {return (float)(data[6]) * 1 / 1;}

    // not used
    uint16_t wbo2_en2_raw() const   {return data[7];}
    uint16_t wbo2_en2_whole() const {return data[7] * 1 / 1;}
    uint16_t wbo2_en2_frac() const  {return data[7] * 1 / 1;}
    float    wbo2_en2_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg02_t
  {
    uint8_t data[8];

    // Barometric pressure
    // units: kPa
    uint16_t baro_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t baro_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t baro_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    baro_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Manifold air pressure
    // units: kPa
    uint16_t map_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t map_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t map_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    map_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Manifold air temperature
    // units: deg F
    uint16_t mat_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t mat_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t mat_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    mat_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Coolant temperature
    // units: deg F
    uint16_t clt_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t clt_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t clt_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    clt_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg03_t
  {
    uint8_t data[8];

    // Throttle position
    // units: %
    uint16_t tps_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t tps_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t tps_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    tps_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Battery voltage
    // units: V
    uint16_t batt_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t batt_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t batt_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    batt_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // AFR1 (Deprecated on MS3)
    // units: AFR
    uint16_t afr1_old_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t afr1_old_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t afr1_old_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    afr1_old_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // AFR2 (Deprecated on MS3)
    // units: AFR
    uint16_t afr2_old_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t afr2_old_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t afr2_old_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    afr2_old_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg04_t
  {
    uint8_t data[8];

    // Indication of knock input
    // units: %
    uint16_t knock_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t knock_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t knock_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    knock_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGO bank 1 correction
    // units: %
    uint16_t egocor1_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egocor1_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t egocor1_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    egocor1_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGO bank2 correction
    // units: %
    uint16_t egocor2_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t egocor2_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t egocor2_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    egocor2_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Air density correction
    // units: %
    uint16_t aircor_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t aircor_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t aircor_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    aircor_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg05_t
  {
    uint8_t data[8];

    // Warmup correction
    // units: %
    uint16_t warmcor_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t warmcor_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t warmcor_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    warmcor_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // TPS-based acceleration
    // units: %
    uint16_t tpsaccel_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t tpsaccel_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t tpsaccel_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    tpsaccel_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // TPS-based fuel cut
    // units: %
    uint16_t tpsfuelcut_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t tpsfuelcut_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t tpsfuelcut_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    tpsfuelcut_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Barometric fuel correction
    // units: %
    uint16_t barocor_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t barocor_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t barocor_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    barocor_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg06_t
  {
    uint8_t data[8];

    // Total fuel correction
    // units: %
    uint16_t totalcor_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t totalcor_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t totalcor_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    totalcor_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // VE value table/bank 1
    // units: %
    uint16_t ve1_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t ve1_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t ve1_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    ve1_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // VE value table/bank 2
    // units: %
    uint16_t ve2_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t ve2_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t ve2_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    ve2_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Stepper idle step number
    // units: step
    uint16_t iacstep_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t iacstep_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t iacstep_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    iacstep_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg07_t
  {
    uint8_t data[8];

    // Cold advance
    // units: deg
    uint16_t cold_adv_deg_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t cold_adv_deg_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t cold_adv_deg_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    cold_adv_deg_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Rate of change of TPS
    // units: %/s
    uint16_t TPSdot_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t TPSdot_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t TPSdot_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    TPSdot_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Rate of change of MAP
    // units: kPa/s
    uint16_t MAPdot_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t MAPdot_whole() const {return MSG_GET_U16(data,4) * 1 / 1;}
    uint16_t MAPdot_frac() const  {return MSG_GET_U16(data,4) * 1 / 1;}
    float    MAPdot_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1;}

    // Rate of change of RPM
    // units: RPM/s
    uint16_t RPMdot_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t RPMdot_whole() const {return MSG_GET_U16(data,6) * 10 / 1;}
    uint16_t RPMdot_frac() const  {return MSG_GET_U16(data,6) * 10 / 1;}
    float    RPMdot_flt() const   {return (float)(MSG_GET_U16(data,6)) * 10 / 1;}
  };

  struct RtMsg08_t
  {
    uint8_t data[8];

    // Synthetic 'load' from MAF
    // units: %
    uint16_t MAFload_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t MAFload_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t MAFload_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    MAFload_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // 'Load' used for fuel table lookup e.g. equals
    // units: %
    uint16_t fuelload_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t fuelload_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t fuelload_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    fuelload_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Adjustment to fuel from Flex
    // units: %
    uint16_t fuelcor_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t fuelcor_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t fuelcor_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    fuelcor_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Mass Air Flow
    // units: g/s
    uint16_t MAF_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t MAF_whole() const {return MSG_GET_U16(data,6) * 1 / 100;}
    uint16_t MAF_frac() const  {return MSG_GET_U16(data,6) * 1 / 100;}
    float    MAF_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 100;}
  };

  struct RtMsg09_t
  {
    uint8_t data[8];

    // Voltage from O2#1 (Deprecated on MS3)
    // units: V
    uint16_t egoV1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t egoV1_whole() const {return MSG_GET_U16(data,0) * 1 / 100;}
    uint16_t egoV1_frac() const  {return MSG_GET_U16(data,0) * 1 / 100;}
    float    egoV1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 100;}

    // Voltage from O2#2 (Deprecated on MS3)
    // units: V
    uint16_t egoV2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egoV2_whole() const {return MSG_GET_U16(data,2) * 1 / 100;}
    uint16_t egoV2_frac() const  {return MSG_GET_U16(data,2) * 1 / 100;}
    float    egoV2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 100;}

    // Main ignition dwell
    // units: ms
    uint16_t dwell_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t dwell_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t dwell_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    dwell_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Trailing ignition dwell
    // units: ms
    uint16_t dwell_trl_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t dwell_trl_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t dwell_trl_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    dwell_trl_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg10_t
  {
    uint8_t data[8];

    // ECU status bitfield
    uint16_t status1_raw() const   {return data[0];}
    uint16_t status1_whole() const {return data[0] * 1 / 1;}
    uint16_t status1_frac() const  {return data[0] * 1 / 1;}
    float    status1_flt() const   {return (float)(data[0]) * 1 / 1;}

    // ECU status bitfield
    uint16_t status2_raw() const   {return data[1];}
    uint16_t status2_whole() const {return data[1] * 1 / 1;}
    uint16_t status2_frac() const  {return data[1] * 1 / 1;}
    float    status2_flt() const   {return (float)(data[1]) * 1 / 1;}

    // ECU status bitfield
    uint16_t status3_raw() const   {return data[2];}
    uint16_t status3_whole() const {return data[2] * 1 / 1;}
    uint16_t status3_frac() const  {return data[2] * 1 / 1;}
    float    status3_flt() const   {return (float)(data[2]) * 1 / 1;}

    // Not typically used
    uint16_t status4_raw() const   {return data[3];}
    uint16_t status4_whole() const {return data[3] * 1 / 1;}
    uint16_t status4_frac() const  {return data[3] * 1 / 1;}
    float    status4_flt() const   {return (float)(data[3]) * 1 / 1;}

    // Not typically used
    uint16_t status5_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t status5_whole() const {return MSG_GET_U16(data,4) * 1 / 1;}
    uint16_t status5_frac() const  {return MSG_GET_U16(data,4) * 1 / 1;}
    float    status5_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1;}

    // ECU status bitfield
    uint16_t status6_raw() const   {return data[6];}
    uint16_t status6_whole() const {return data[6] * 1 / 1;}
    uint16_t status6_frac() const  {return data[6] * 1 / 1;}
    float    status6_flt() const   {return (float)(data[6]) * 1 / 1;}

    // ECU status bitfield
    uint16_t status7_raw() const   {return data[7];}
    uint16_t status7_whole() const {return data[7] * 1 / 1;}
    uint16_t status7_frac() const  {return data[7] * 1 / 1;}
    float    status7_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg11_t
  {
    uint8_t data[8];

    // 'Load' used for fuelling on modified table
    // units: %
    uint16_t fuelload2_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t fuelload2_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t fuelload2_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    fuelload2_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // 'Load' used for ignition table lookup
    // units: %
    uint16_t ignload_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t ignload_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t ignload_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    ignload_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // 'Load' used for modifier ignition table lookup
    // units: %
    uint16_t ignload2_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t ignload2_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t ignload2_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    ignload2_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Estimated intake air temperature
    // units: deg F
    uint16_t airtemp_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t airtemp_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t airtemp_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    airtemp_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg12_t
  {
    uint8_t data[8];

    // Calculated volume of fuel on intake walls from
    // units: us
    uint16_t wallfuel1_raw() const   {return MSG_GET_U32(data,0);}
    uint16_t wallfuel1_whole() const {return MSG_GET_U32(data,0) * 1 / 100;}
    uint16_t wallfuel1_frac() const  {return MSG_GET_U32(data,0) * 1 / 100;}
    float    wallfuel1_flt() const   {return (float)(MSG_GET_U32(data,0)) * 1 / 100;}

    // Second channel of same
    // units: us
    uint16_t wallfuel2_raw() const   {return MSG_GET_U32(data,4);}
    uint16_t wallfuel2_whole() const {return MSG_GET_U32(data,4) * 1 / 100;}
    uint16_t wallfuel2_frac() const  {return MSG_GET_U32(data,4) * 1 / 100;}
    float    wallfuel2_flt() const   {return (float)(MSG_GET_U32(data,4)) * 1 / 100;}
  };

  struct RtMsg13_t
  {
    uint8_t data[8];

    // Generic sensor input 1 (gpioadc0 on MS2)
    uint16_t sensors1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t sensors1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t sensors1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    sensors1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Generic sensor input 2 (gpioadc1 on MS2)
    uint16_t sensors2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t sensors2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t sensors2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    sensors2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Generic sensor input 3 (gpioadc2 on MS2)
    uint16_t sensors3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t sensors3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t sensors3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    sensors3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Generic sensor input 4 (gpioadc3 on MS2)
    uint16_t sensors4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t sensors4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t sensors4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    sensors4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg14_t
  {
    uint8_t data[8];

    // Generic sensor input 5 (gpioadc4 on MS2)
    uint16_t sensors5_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t sensors5_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t sensors5_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    sensors5_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Generic sensor input 6 (gpioadc5 on MS2)
    uint16_t sensors6_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t sensors6_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t sensors6_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    sensors6_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Generic sensor input 7 (gpioadc6 on MS2)
    uint16_t sensors7_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t sensors7_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t sensors7_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    sensors7_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Generic sensor input 8 (gpioadc7 on MS2)
    uint16_t sensors8_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t sensors8_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t sensors8_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    sensors8_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg15_t
  {
    uint8_t data[8];

    // Generic sensor input 9 (adc6 on MS2)
    uint16_t sensors9_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t sensors9_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t sensors9_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    sensors9_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Generic sensor input 10 (adc7 on MS2)
    uint16_t sensors10_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t sensors10_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t sensors10_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    sensors10_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Generic sensor input 11
    uint16_t sensors11_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t sensors11_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t sensors11_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    sensors11_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Generic sensor input 12
    uint16_t sensors12_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t sensors12_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t sensors12_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    sensors12_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg16_t
  {
    uint8_t data[8];

    // Generic sensor input 13
    uint16_t sensors13_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t sensors13_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t sensors13_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    sensors13_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Generic sensor input 14
    uint16_t sensors14_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t sensors14_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t sensors14_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    sensors14_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Generic sensor input 15
    uint16_t sensors15_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t sensors15_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t sensors15_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    sensors15_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Generic sensor input 16
    uint16_t sensors16_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t sensors16_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t sensors16_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    sensors16_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg17_t
  {
    uint8_t data[8];

    // Target boost - channel 1
    // units: kPa
    uint16_t boost_targ_1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t boost_targ_1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t boost_targ_1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    boost_targ_1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Target boost - channel 2
    // units: kPa
    uint16_t boost_targ_2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t boost_targ_2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t boost_targ_2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    boost_targ_2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Duty cycle on boost solenoid 1
    // units: %
    uint16_t boostduty_raw() const   {return data[4];}
    uint16_t boostduty_whole() const {return data[4] * 1 / 1;}
    uint16_t boostduty_frac() const  {return data[4] * 1 / 1;}
    float    boostduty_flt() const   {return (float)(data[4]) * 1 / 1;}

    // Duty cycle on boost solenoid 2
    // units: %
    uint16_t boostduty2_raw() const   {return data[5];}
    uint16_t boostduty2_whole() const {return data[5] * 1 / 1;}
    uint16_t boostduty2_frac() const  {return data[5] * 1 / 1;}
    float    boostduty2_flt() const   {return (float)(data[5]) * 1 / 1;}

    // MAF voltage (synthesised for frequency
    // units: V
    uint16_t maf_volts_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t maf_volts_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t maf_volts_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    maf_volts_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg18_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#1
    // units: ms
    uint16_t pwseq1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t pwseq1_whole() const {return MSG_GET_U16(data,0) * 1 / 1000;}
    uint16_t pwseq1_frac() const  {return MSG_GET_U16(data,0) * 1 / 1000;}
    float    pwseq1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#2
    // units: ms
    uint16_t pwseq2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t pwseq2_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t pwseq2_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    pwseq2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#3
    // units: ms
    uint16_t pwseq3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t pwseq3_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t pwseq3_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    pwseq3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#4
    // units: ms
    uint16_t pwseq4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t pwseq4_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t pwseq4_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    pwseq4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg19_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#5
    // units: ms
    uint16_t pwseq5_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t pwseq5_whole() const {return MSG_GET_U16(data,0) * 1 / 1000;}
    uint16_t pwseq5_frac() const  {return MSG_GET_U16(data,0) * 1 / 1000;}
    float    pwseq5_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#6
    // units: ms
    uint16_t pwseq6_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t pwseq6_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t pwseq6_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    pwseq6_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#7
    // units: ms
    uint16_t pwseq7_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t pwseq7_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t pwseq7_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    pwseq7_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#8
    // units: ms
    uint16_t pwseq8_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t pwseq8_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t pwseq8_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    pwseq8_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg20_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#9
    // units: ms
    uint16_t pwseq9_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t pwseq9_whole() const {return MSG_GET_U16(data,0) * 1 / 1000;}
    uint16_t pwseq9_frac() const  {return MSG_GET_U16(data,0) * 1 / 1000;}
    float    pwseq9_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#10
    // units: ms
    uint16_t pwseq10_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t pwseq10_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t pwseq10_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    pwseq10_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#11
    // units: ms
    uint16_t pwseq11_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t pwseq11_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t pwseq11_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    pwseq11_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#12
    // units: ms
    uint16_t pwseq12_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t pwseq12_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t pwseq12_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    pwseq12_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg21_t
  {
    uint8_t data[8];

    // Sequential Pulsewidth for cyl#13
    // units: ms
    uint16_t pwseq13_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t pwseq13_whole() const {return MSG_GET_U16(data,0) * 1 / 1000;}
    uint16_t pwseq13_frac() const  {return MSG_GET_U16(data,0) * 1 / 1000;}
    float    pwseq13_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#14
    // units: ms
    uint16_t pwseq14_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t pwseq14_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t pwseq14_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    pwseq14_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#15
    // units: ms
    uint16_t pwseq15_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t pwseq15_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t pwseq15_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    pwseq15_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Sequential Pulsewidth for cyl#16
    // units: ms
    uint16_t pwseq16_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t pwseq16_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t pwseq16_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    pwseq16_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg22_t
  {
    uint8_t data[8];

    // EGT 1
    // units: deg F
    uint16_t egt1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t egt1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t egt1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    egt1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGT 2
    // units: deg F
    uint16_t egt2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egt2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t egt2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    egt2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGT 3
    // units: deg F
    uint16_t egt3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t egt3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t egt3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    egt3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGT 4
    // units: deg F
    uint16_t egt4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t egt4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t egt4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    egt4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg23_t
  {
    uint8_t data[8];

    // EGT 5
    // units: deg F
    uint16_t egt5_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t egt5_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t egt5_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    egt5_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGT 6
    // units: deg F
    uint16_t egt6_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egt6_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t egt6_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    egt6_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGT 7
    // units: deg F
    uint16_t egt7_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t egt7_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t egt7_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    egt7_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGT 8
    // units: deg F
    uint16_t egt8_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t egt8_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t egt8_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    egt8_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg24_t
  {
    uint8_t data[8];

    // EGT 9
    // units: deg F
    uint16_t egt9_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t egt9_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t egt9_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    egt9_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGT 10
    // units: deg F
    uint16_t egt10_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egt10_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t egt10_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    egt10_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGT 11
    // units: deg F
    uint16_t egt11_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t egt11_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t egt11_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    egt11_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGT 12
    // units: deg F
    uint16_t egt12_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t egt12_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t egt12_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    egt12_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg25_t
  {
    uint8_t data[8];

    // EGT 13
    // units: deg F
    uint16_t egt13_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t egt13_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t egt13_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    egt13_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGT 14
    // units: deg F
    uint16_t egt14_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t egt14_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t egt14_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    egt14_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGT 15
    // units: deg F
    uint16_t egt15_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t egt15_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t egt15_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    egt15_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGT 16
    // units: deg F
    uint16_t egt16_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t egt16_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t egt16_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    egt16_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg26_t
  {
    uint8_t data[8];

    // Duty cycle to nitrous solenoid 1
    // units: %
    uint16_t nitrous1_duty_raw() const   {return data[0];}
    uint16_t nitrous1_duty_whole() const {return data[0] * 1 / 1;}
    uint16_t nitrous1_duty_frac() const  {return data[0] * 1 / 1;}
    float    nitrous1_duty_flt() const   {return (float)(data[0]) * 1 / 1;}

    // Duty cycle to nitrous solenoid 2
    // units: %
    uint16_t nitrous2_duty_raw() const   {return data[1];}
    uint16_t nitrous2_duty_whole() const {return data[1] * 1 / 1;}
    uint16_t nitrous2_duty_frac() const  {return data[1] * 1 / 1;}
    float    nitrous2_duty_flt() const   {return (float)(data[1]) * 1 / 1;}

    // Timer used internally for nitrous system
    // units: s
    uint16_t nitrous_timer_ou_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t nitrous_timer_ou_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t nitrous_timer_ou_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    nitrous_timer_ou_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // Fuel pulsewidth added due to nitrous system
    // units: ms
    uint16_t n2o_addfuel_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t n2o_addfuel_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t n2o_addfuel_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    n2o_addfuel_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Timing retard due to nitrous system
    // units: deg
    uint16_t n2o_retard_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t n2o_retard_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t n2o_retard_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    n2o_retard_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg27_t
  {
    uint8_t data[8];

    // PWM period 1 from remote board
    uint16_t canpwmin1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t canpwmin1_whole() const {return MSG_GET_U16(data,0) * 1 / 1;}
    uint16_t canpwmin1_frac() const  {return MSG_GET_U16(data,0) * 1 / 1;}
    float    canpwmin1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1;}

    // PWM period 2 from remote board
    uint16_t canpwmin2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t canpwmin2_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t canpwmin2_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    canpwmin2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // PWM period 3 from remote board
    uint16_t canpwmin3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t canpwmin3_whole() const {return MSG_GET_U16(data,4) * 1 / 1;}
    uint16_t canpwmin3_frac() const  {return MSG_GET_U16(data,4) * 1 / 1;}
    float    canpwmin3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1;}

    // PWM period 4 from remote board
    uint16_t canpwmin4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t canpwmin4_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t canpwmin4_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    canpwmin4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg28_t
  {
    uint8_t data[8];

    // Closed-loop idle target RPM
    // units: RPM
    uint16_t cl_idle_targ_rpm_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t cl_idle_targ_rpm_whole() const {return MSG_GET_U16(data,0) * 1 / 1;}
    uint16_t cl_idle_targ_rpm_frac() const  {return MSG_GET_U16(data,0) * 1 / 1;}
    float    cl_idle_targ_rpm_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1;}

    // ADC count from TPS
    uint16_t tpsadc_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t tpsadc_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t tpsadc_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    tpsadc_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // 'Load' used for EAE calc
    // units: %
    uint16_t eaeload_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t eaeload_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t eaeload_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    eaeload_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // 'Load' used for AFR table lookups
    // units: %
    uint16_t afrload_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t afrload_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t afrload_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    afrload_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg29_t
  {
    uint8_t data[8];

    // Fuel correction from EAE - channel 1
    // units: %
    uint16_t EAEfcor1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EAEfcor1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t EAEfcor1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    EAEfcor1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Fuel correction from EAE - channel 2
    // units: %
    uint16_t EAEfcor2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EAEfcor2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t EAEfcor2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    EAEfcor2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Rate of change of VSS1
    // units: ms-2
    uint16_t VSS1dot_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t VSS1dot_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t VSS1dot_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    VSS1dot_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Rate of change of VSS2
    // units: ms-2
    uint16_t VSS2dot_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t VSS2dot_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t VSS2dot_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    VSS2dot_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg30_t
  {
    uint8_t data[8];

    // External accelerometer X
    // units: ms-2
    uint16_t accelx_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t accelx_whole() const {return MSG_GET_U16(data,0) * 1 / 1000;}
    uint16_t accelx_frac() const  {return MSG_GET_U16(data,0) * 1 / 1000;}
    float    accelx_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1000;}

    // External accelerometer Y
    // units: ms-2
    uint16_t accely_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t accely_whole() const {return MSG_GET_U16(data,2) * 1 / 1000;}
    uint16_t accely_frac() const  {return MSG_GET_U16(data,2) * 1 / 1000;}
    float    accely_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1000;}

    // External accelerometer Z
    // units: ms-2
    uint16_t accelz_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t accelz_whole() const {return MSG_GET_U16(data,4) * 1 / 1000;}
    uint16_t accelz_frac() const  {return MSG_GET_U16(data,4) * 1 / 1000;}
    float    accelz_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1000;}

    // Volume level on audio input
    uint16_t stream_level_raw() const   {return data[6];}
    uint16_t stream_level_whole() const {return data[6] * 1 / 1;}
    uint16_t stream_level_frac() const  {return data[6] * 1 / 1;}
    float    stream_level_flt() const   {return (float)(data[6]) * 1 / 1;}

    // Duty cycle to water injection solenoid
    // units: %
    uint16_t water_duty_raw() const   {return data[7];}
    uint16_t water_duty_whole() const {return data[7] * 1 / 1;}
    uint16_t water_duty_frac() const  {return data[7] * 1 / 1;}
    float    water_duty_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg31_t
  {
    uint8_t data[8];

    // AFR cyl#1
    // units: AFR
    uint16_t AFR1_raw() const   {return data[0];}
    uint16_t AFR1_whole() const {return data[0] * 1 / 10;}
    uint16_t AFR1_frac() const  {return data[0] * 1 / 10;}
    float    AFR1_flt() const   {return (float)(data[0]) * 1 / 10;}

    // AFR cyl#2
    // units: AFR
    uint16_t AFR2_raw() const   {return data[1];}
    uint16_t AFR2_whole() const {return data[1] * 1 / 10;}
    uint16_t AFR2_frac() const  {return data[1] * 1 / 10;}
    float    AFR2_flt() const   {return (float)(data[1]) * 1 / 10;}

    // AFR cyl#3
    // units: AFR
    uint16_t AFR3_raw() const   {return data[2];}
    uint16_t AFR3_whole() const {return data[2] * 1 / 10;}
    uint16_t AFR3_frac() const  {return data[2] * 1 / 10;}
    float    AFR3_flt() const   {return (float)(data[2]) * 1 / 10;}

    // AFR cyl#4
    // units: AFR
    uint16_t AFR4_raw() const   {return data[3];}
    uint16_t AFR4_whole() const {return data[3] * 1 / 10;}
    uint16_t AFR4_frac() const  {return data[3] * 1 / 10;}
    float    AFR4_flt() const   {return (float)(data[3]) * 1 / 10;}

    // AFR cyl#5
    // units: AFR
    uint16_t AFR5_raw() const   {return data[4];}
    uint16_t AFR5_whole() const {return data[4] * 1 / 10;}
    uint16_t AFR5_frac() const  {return data[4] * 1 / 10;}
    float    AFR5_flt() const   {return (float)(data[4]) * 1 / 10;}

    // AFR cyl#6
    // units: AFR
    uint16_t AFR6_raw() const   {return data[5];}
    uint16_t AFR6_whole() const {return data[5] * 1 / 10;}
    uint16_t AFR6_frac() const  {return data[5] * 1 / 10;}
    float    AFR6_flt() const   {return (float)(data[5]) * 1 / 10;}

    // AFR cyl#7
    // units: AFR
    uint16_t AFR7_raw() const   {return data[6];}
    uint16_t AFR7_whole() const {return data[6] * 1 / 10;}
    uint16_t AFR7_frac() const  {return data[6] * 1 / 10;}
    float    AFR7_flt() const   {return (float)(data[6]) * 1 / 10;}

    // AFR cyl#8
    // units: AFR
    uint16_t AFR8_raw() const   {return data[7];}
    uint16_t AFR8_whole() const {return data[7] * 1 / 10;}
    uint16_t AFR8_frac() const  {return data[7] * 1 / 10;}
    float    AFR8_flt() const   {return (float)(data[7]) * 1 / 10;}
  };

  struct RtMsg32_t
  {
    uint8_t data[8];

    // AFR cyl#9
    // units: AFR
    uint16_t AFR9_raw() const   {return data[0];}
    uint16_t AFR9_whole() const {return data[0] * 1 / 10;}
    uint16_t AFR9_frac() const  {return data[0] * 1 / 10;}
    float    AFR9_flt() const   {return (float)(data[0]) * 1 / 10;}

    // AFR cyl#10
    // units: AFR
    uint16_t AFR10_raw() const   {return data[1];}
    uint16_t AFR10_whole() const {return data[1] * 1 / 10;}
    uint16_t AFR10_frac() const  {return data[1] * 1 / 10;}
    float    AFR10_flt() const   {return (float)(data[1]) * 1 / 10;}

    // AFR cyl#11
    // units: AFR
    uint16_t AFR11_raw() const   {return data[2];}
    uint16_t AFR11_whole() const {return data[2] * 1 / 10;}
    uint16_t AFR11_frac() const  {return data[2] * 1 / 10;}
    float    AFR11_flt() const   {return (float)(data[2]) * 1 / 10;}

    // AFR cyl#12
    // units: AFR
    uint16_t AFR12_raw() const   {return data[3];}
    uint16_t AFR12_whole() const {return data[3] * 1 / 10;}
    uint16_t AFR12_frac() const  {return data[3] * 1 / 10;}
    float    AFR12_flt() const   {return (float)(data[3]) * 1 / 10;}

    // AFR cyl#13
    // units: AFR
    uint16_t AFR13_raw() const   {return data[4];}
    uint16_t AFR13_whole() const {return data[4] * 1 / 10;}
    uint16_t AFR13_frac() const  {return data[4] * 1 / 10;}
    float    AFR13_flt() const   {return (float)(data[4]) * 1 / 10;}

    // AFR cyl#14
    // units: AFR
    uint16_t AFR14_raw() const   {return data[5];}
    uint16_t AFR14_whole() const {return data[5] * 1 / 10;}
    uint16_t AFR14_frac() const  {return data[5] * 1 / 10;}
    float    AFR14_flt() const   {return (float)(data[5]) * 1 / 10;}

    // AFR cyl#15
    // units: AFR
    uint16_t AFR15_raw() const   {return data[6];}
    uint16_t AFR15_whole() const {return data[6] * 1 / 10;}
    uint16_t AFR15_frac() const  {return data[6] * 1 / 10;}
    float    AFR15_flt() const   {return (float)(data[6]) * 1 / 10;}

    // AFR cyl#16
    // units: AFR
    uint16_t AFR16_raw() const   {return data[7];}
    uint16_t AFR16_whole() const {return data[7] * 1 / 10;}
    uint16_t AFR16_frac() const  {return data[7] * 1 / 10;}
    float    AFR16_flt() const   {return (float)(data[7]) * 1 / 10;}
  };

  struct RtMsg33_t
  {
    uint8_t data[8];

    // Generic PWM duty 1
    // units: %
    uint16_t duty_pwm1_raw() const   {return data[0];}
    uint16_t duty_pwm1_whole() const {return data[0] * 1 / 1;}
    uint16_t duty_pwm1_frac() const  {return data[0] * 1 / 1;}
    float    duty_pwm1_flt() const   {return (float)(data[0]) * 1 / 1;}

    // Generic PWM duty 2
    // units: %
    uint16_t duty_pwm2_raw() const   {return data[1];}
    uint16_t duty_pwm2_whole() const {return data[1] * 1 / 1;}
    uint16_t duty_pwm2_frac() const  {return data[1] * 1 / 1;}
    float    duty_pwm2_flt() const   {return (float)(data[1]) * 1 / 1;}

    // Generic PWM duty 3
    // units: %
    uint16_t duty_pwm3_raw() const   {return data[2];}
    uint16_t duty_pwm3_whole() const {return data[2] * 1 / 1;}
    uint16_t duty_pwm3_frac() const  {return data[2] * 1 / 1;}
    float    duty_pwm3_flt() const   {return (float)(data[2]) * 1 / 1;}

    // Generic PWM duty 4
    // units: %
    uint16_t duty_pwm4_raw() const   {return data[3];}
    uint16_t duty_pwm4_whole() const {return data[3] * 1 / 1;}
    uint16_t duty_pwm4_frac() const  {return data[3] * 1 / 1;}
    float    duty_pwm4_flt() const   {return (float)(data[3]) * 1 / 1;}

    // Generic PWM duty 5
    // units: %
    uint16_t duty_pwm5_raw() const   {return data[4];}
    uint16_t duty_pwm5_whole() const {return data[4] * 1 / 1;}
    uint16_t duty_pwm5_frac() const  {return data[4] * 1 / 1;}
    float    duty_pwm5_flt() const   {return (float)(data[4]) * 1 / 1;}

    // Generic PWM duty 6
    // units: %
    uint16_t duty_pwm6_raw() const   {return data[5];}
    uint16_t duty_pwm6_whole() const {return data[5] * 1 / 1;}
    uint16_t duty_pwm6_frac() const  {return data[5] * 1 / 1;}
    float    duty_pwm6_flt() const   {return (float)(data[5]) * 1 / 1;}

    // Current gear selected
    uint16_t gear_raw() const   {return data[6];}
    uint16_t gear_whole() const {return data[6] * 1 / 1;}
    uint16_t gear_frac() const  {return data[6] * 1 / 1;}
    float    gear_flt() const   {return (float)(data[6]) * 1 / 1;}

    // Engine status bitfield
    uint16_t status8_raw() const   {return data[7];}
    uint16_t status8_whole() const {return data[7] * 1 / 1;}
    uint16_t status8_frac() const  {return data[7] * 1 / 1;}
    float    status8_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg34_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#1
    // units: V
    uint16_t EGOv1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOv1_whole() const {return MSG_GET_U16(data,0) * 489 / 100000;}
    uint16_t EGOv1_frac() const  {return MSG_GET_U16(data,0) * 489 / 100000;}
    float    EGOv1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 489 / 100000;}

    // Voltage from O2 cyl#2
    // units: V
    uint16_t EGOv2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOv2_whole() const {return MSG_GET_U16(data,2) * 489 / 100000;}
    uint16_t EGOv2_frac() const  {return MSG_GET_U16(data,2) * 489 / 100000;}
    float    EGOv2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 489 / 100000;}

    // Voltage from O2 cyl#3
    // units: V
    uint16_t EGOv3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOv3_whole() const {return MSG_GET_U16(data,4) * 489 / 100000;}
    uint16_t EGOv3_frac() const  {return MSG_GET_U16(data,4) * 489 / 100000;}
    float    EGOv3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 489 / 100000;}

    // Voltage from O2 cyl#4
    // units: V
    uint16_t EGOv4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOv4_whole() const {return MSG_GET_U16(data,6) * 489 / 100000;}
    uint16_t EGOv4_frac() const  {return MSG_GET_U16(data,6) * 489 / 100000;}
    float    EGOv4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 489 / 100000;}
  };

  struct RtMsg35_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#5
    // units: V
    uint16_t EGOv5_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOv5_whole() const {return MSG_GET_U16(data,0) * 489 / 100000;}
    uint16_t EGOv5_frac() const  {return MSG_GET_U16(data,0) * 489 / 100000;}
    float    EGOv5_flt() const   {return (float)(MSG_GET_U16(data,0)) * 489 / 100000;}

    // Voltage from O2 cyl#6
    // units: V
    uint16_t EGOv6_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOv6_whole() const {return MSG_GET_U16(data,2) * 489 / 100000;}
    uint16_t EGOv6_frac() const  {return MSG_GET_U16(data,2) * 489 / 100000;}
    float    EGOv6_flt() const   {return (float)(MSG_GET_U16(data,2)) * 489 / 100000;}

    // Voltage from O2 cyl#7
    // units: V
    uint16_t EGOv7_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOv7_whole() const {return MSG_GET_U16(data,4) * 489 / 100000;}
    uint16_t EGOv7_frac() const  {return MSG_GET_U16(data,4) * 489 / 100000;}
    float    EGOv7_flt() const   {return (float)(MSG_GET_U16(data,4)) * 489 / 100000;}

    // Voltage from O2 cyl#8
    // units: V
    uint16_t EGOv8_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOv8_whole() const {return MSG_GET_U16(data,6) * 489 / 100000;}
    uint16_t EGOv8_frac() const  {return MSG_GET_U16(data,6) * 489 / 100000;}
    float    EGOv8_flt() const   {return (float)(MSG_GET_U16(data,6)) * 489 / 100000;}
  };

  struct RtMsg36_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#9
    // units: V
    uint16_t EGOv9_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOv9_whole() const {return MSG_GET_U16(data,0) * 489 / 100000;}
    uint16_t EGOv9_frac() const  {return MSG_GET_U16(data,0) * 489 / 100000;}
    float    EGOv9_flt() const   {return (float)(MSG_GET_U16(data,0)) * 489 / 100000;}

    // Voltage from O2 cyl#10
    // units: V
    uint16_t EGOv10_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOv10_whole() const {return MSG_GET_U16(data,2) * 489 / 100000;}
    uint16_t EGOv10_frac() const  {return MSG_GET_U16(data,2) * 489 / 100000;}
    float    EGOv10_flt() const   {return (float)(MSG_GET_U16(data,2)) * 489 / 100000;}

    // Voltage from O2 cyl#11
    // units: V
    uint16_t EGOv11_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOv11_whole() const {return MSG_GET_U16(data,4) * 489 / 100000;}
    uint16_t EGOv11_frac() const  {return MSG_GET_U16(data,4) * 489 / 100000;}
    float    EGOv11_flt() const   {return (float)(MSG_GET_U16(data,4)) * 489 / 100000;}

    // Voltage from O2 cyl#12
    // units: V
    uint16_t EGOv12_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOv12_whole() const {return MSG_GET_U16(data,6) * 489 / 100000;}
    uint16_t EGOv12_frac() const  {return MSG_GET_U16(data,6) * 489 / 100000;}
    float    EGOv12_flt() const   {return (float)(MSG_GET_U16(data,6)) * 489 / 100000;}
  };

  struct RtMsg37_t
  {
    uint8_t data[8];

    // Voltage from O2 cyl#13
    // units: V
    uint16_t EGOv13_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOv13_whole() const {return MSG_GET_U16(data,0) * 489 / 100000;}
    uint16_t EGOv13_frac() const  {return MSG_GET_U16(data,0) * 489 / 100000;}
    float    EGOv13_flt() const   {return (float)(MSG_GET_U16(data,0)) * 489 / 100000;}

    // Voltage from O2 cyl#14
    // units: V
    uint16_t EGOv14_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOv14_whole() const {return MSG_GET_U16(data,2) * 489 / 100000;}
    uint16_t EGOv14_frac() const  {return MSG_GET_U16(data,2) * 489 / 100000;}
    float    EGOv14_flt() const   {return (float)(MSG_GET_U16(data,2)) * 489 / 100000;}

    // Voltage from O2 cyl#15
    // units: V
    uint16_t EGOv15_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOv15_whole() const {return MSG_GET_U16(data,4) * 489 / 100000;}
    uint16_t EGOv15_frac() const  {return MSG_GET_U16(data,4) * 489 / 100000;}
    float    EGOv15_flt() const   {return (float)(MSG_GET_U16(data,4)) * 489 / 100000;}

    // Voltage from O2 cyl#16
    // units: V
    uint16_t EGOv16_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOv16_whole() const {return MSG_GET_U16(data,6) * 489 / 100000;}
    uint16_t EGOv16_frac() const  {return MSG_GET_U16(data,6) * 489 / 100000;}
    float    EGOv16_flt() const   {return (float)(MSG_GET_U16(data,6)) * 489 / 100000;}
  };

  struct RtMsg38_t
  {
    uint8_t data[8];

    // EGO correction cyl#1
    // units: %
    uint16_t EGOcor1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOcor1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t EGOcor1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    EGOcor1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGO correction cyl#2
    // units: %
    uint16_t EGOcor2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOcor2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t EGOcor2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    EGOcor2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGO correction cyl#3
    // units: %
    uint16_t EGOcor3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOcor3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t EGOcor3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    EGOcor3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGO correction cyl#4
    // units: %
    uint16_t EGOcor4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOcor4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t EGOcor4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    EGOcor4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg39_t
  {
    uint8_t data[8];

    // EGO correction cyl#5
    // units: %
    uint16_t EGOcor5_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOcor5_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t EGOcor5_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    EGOcor5_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGO correction cyl#6
    // units: %
    uint16_t EGOcor6_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOcor6_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t EGOcor6_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    EGOcor6_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGO correction cyl#7
    // units: %
    uint16_t EGOcor7_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOcor7_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t EGOcor7_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    EGOcor7_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGO correction cyl#8
    // units: %
    uint16_t EGOcor8_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOcor8_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t EGOcor8_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    EGOcor8_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg40_t
  {
    uint8_t data[8];

    // EGO correction cyl#9
    // units: %
    uint16_t EGOcor9_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOcor9_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t EGOcor9_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    EGOcor9_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGO correction cyl#10
    // units: %
    uint16_t EGOcor10_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOcor10_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t EGOcor10_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    EGOcor10_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGO correction cyl#11
    // units: %
    uint16_t EGOcor11_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOcor11_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t EGOcor11_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    EGOcor11_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGO correction cyl#12
    // units: %
    uint16_t EGOcor12_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOcor12_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t EGOcor12_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    EGOcor12_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg41_t
  {
    uint8_t data[8];

    // EGO correction cyl#13
    // units: %
    uint16_t EGOcor13_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t EGOcor13_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t EGOcor13_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    EGOcor13_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // EGO correction cyl#14
    // units: %
    uint16_t EGOcor14_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t EGOcor14_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t EGOcor14_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    EGOcor14_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // EGO correction cyl#15
    // units: %
    uint16_t EGOcor15_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t EGOcor15_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t EGOcor15_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    EGOcor15_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // EGO correction cyl#16
    // units: %
    uint16_t EGOcor16_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t EGOcor16_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t EGOcor16_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    EGOcor16_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg42_t
  {
    uint8_t data[8];

    // Vehicle Speed 1
    // units: ms-1
    uint16_t VSS1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t VSS1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t VSS1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    VSS1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Vehicle Speed 2
    // units: ms-1
    uint16_t VSS2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t VSS2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t VSS2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    VSS2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Vehicle Speed 3
    // units: ms-1
    uint16_t VSS3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t VSS3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t VSS3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    VSS3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Vehicle Speed 4
    // units: ms-1
    uint16_t VSS4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t VSS4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t VSS4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    VSS4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg43_t
  {
    uint8_t data[8];

    // Sync-loss counter
    uint16_t synccnt_raw() const   {return data[0];}
    uint16_t synccnt_whole() const {return data[0] * 1 / 1;}
    uint16_t synccnt_frac() const  {return data[0] * 1 / 1;}
    float    synccnt_flt() const   {return (float)(data[0]) * 1 / 1;}

    // Sync-loss reason code
    uint16_t syncreason_raw() const   {return data[1];}
    uint16_t syncreason_whole() const {return data[1] * 1 / 1;}
    uint16_t syncreason_frac() const  {return data[1] * 1 / 1;}
    float    syncreason_flt() const   {return (float)(data[1]) * 1 / 1;}

    // SDcard file number
    uint16_t sd_filenum_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t sd_filenum_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t sd_filenum_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    sd_filenum_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // SDcard error number
    uint16_t sd_error_raw() const   {return data[4];}
    uint16_t sd_error_whole() const {return data[4] * 1 / 1;}
    uint16_t sd_error_frac() const  {return data[4] * 1 / 1;}
    float    sd_error_flt() const   {return (float)(data[4]) * 1 / 1;}

    // SDcard internal code
    uint16_t sd_phase_raw() const   {return data[5];}
    uint16_t sd_phase_whole() const {return data[5] * 1 / 1;}
    uint16_t sd_phase_frac() const  {return data[5] * 1 / 1;}
    float    sd_phase_flt() const   {return (float)(data[5]) * 1 / 1;}

    // SDcard status bitfield
    uint16_t sd_status_raw() const   {return data[6];}
    uint16_t sd_status_whole() const {return data[6] * 1 / 1;}
    uint16_t sd_status_frac() const  {return data[6] * 1 / 1;}
    float    sd_status_flt() const   {return (float)(data[6]) * 1 / 1;}

    // Calculated error in ignition timing
    // units: %
    uint16_t timing_err_raw() const   {return data[7];}
    uint16_t timing_err_whole() const {return data[7] * 1 / 1;}
    uint16_t timing_err_frac() const  {return data[7] * 1 / 1;}
    float    timing_err_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg44_t
  {
    uint8_t data[8];

    // VVT actual angle 1
    // units: deg
    uint16_t vvt_ang1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t vvt_ang1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t vvt_ang1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    vvt_ang1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // VVT actual angle 2
    // units: deg
    uint16_t vvt_ang2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t vvt_ang2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t vvt_ang2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    vvt_ang2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // VVT actual angle 3
    // units: deg
    uint16_t vvt_ang3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t vvt_ang3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t vvt_ang3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    vvt_ang3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // VVT actual angle 4
    // units: deg
    uint16_t vvt_ang4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t vvt_ang4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t vvt_ang4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    vvt_ang4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg45_t
  {
    uint8_t data[8];

    // VVT target angle 1
    // units: deg
    uint16_t vvt_target1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t vvt_target1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t vvt_target1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    vvt_target1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // VVT target angle 2
    // units: deg
    uint16_t vvt_target2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t vvt_target2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t vvt_target2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    vvt_target2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // VVT target angle 3
    // units: deg
    uint16_t vvt_target3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t vvt_target3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t vvt_target3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    vvt_target3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // VVT target angle 4
    // units: deg
    uint16_t vvt_target4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t vvt_target4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t vvt_target4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    vvt_target4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg46_t
  {
    uint8_t data[8];

    // VVT solenoid 1 duty cycle
    // units: %
    uint16_t vvt_duty1_raw() const   {return data[0];}
    uint16_t vvt_duty1_whole() const {return data[0] * 392 / 1000;}
    uint16_t vvt_duty1_frac() const  {return data[0] * 392 / 1000;}
    float    vvt_duty1_flt() const   {return (float)(data[0]) * 392 / 1000;}

    // VVT solenoid 2 duty cycle
    // units: %
    uint16_t vvt_duty2_raw() const   {return data[1];}
    uint16_t vvt_duty2_whole() const {return data[1] * 392 / 1000;}
    uint16_t vvt_duty2_frac() const  {return data[1] * 392 / 1000;}
    float    vvt_duty2_flt() const   {return (float)(data[1]) * 392 / 1000;}

    // VVT solenoid 3 duty cycle
    // units: %
    uint16_t vvt_duty3_raw() const   {return data[2];}
    uint16_t vvt_duty3_whole() const {return data[2] * 392 / 1000;}
    uint16_t vvt_duty3_frac() const  {return data[2] * 392 / 1000;}
    float    vvt_duty3_flt() const   {return (float)(data[2]) * 392 / 1000;}

    // VVT solenoid 4 duty cycle
    // units: %
    uint16_t vvt_duty4_raw() const   {return data[3];}
    uint16_t vvt_duty4_whole() const {return data[3] * 392 / 1000;}
    uint16_t vvt_duty4_frac() const  {return data[3] * 392 / 1000;}
    float    vvt_duty4_flt() const   {return (float)(data[3]) * 392 / 1000;}

    // Injection Timing Angle (primary)
    // units: deg BTDC
    uint16_t inj_timing_pri_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t inj_timing_pri_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t inj_timing_pri_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    inj_timing_pri_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Injection Timing Angle (secondary)
    // units: deg BTDC
    uint16_t inj_timing_sec_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t inj_timing_sec_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t inj_timing_sec_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    inj_timing_sec_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg47_t
  {
    uint8_t data[8];

    // Ethanol content of fuel from Flex sensor
    // units: %
    uint16_t fuel_pct_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t fuel_pct_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t fuel_pct_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    fuel_pct_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // TPSdot based accel
    // units: %
    uint16_t tps_accel_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t tps_accel_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t tps_accel_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    tps_accel_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Shaft speed 1
    // units: RPM
    uint16_t SS1_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t SS1_whole() const {return MSG_GET_U16(data,4) * 10 / 1;}
    uint16_t SS1_frac() const  {return MSG_GET_U16(data,4) * 10 / 1;}
    float    SS1_flt() const   {return (float)(MSG_GET_U16(data,4)) * 10 / 1;}

    // Shaft speed 2
    // units: RPM
    uint16_t SS2_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t SS2_whole() const {return MSG_GET_U16(data,6) * 10 / 1;}
    uint16_t SS2_frac() const  {return MSG_GET_U16(data,6) * 10 / 1;}
    float    SS2_flt() const   {return (float)(MSG_GET_U16(data,6)) * 10 / 1;}
  };

  struct RtMsg48_t
  {
    uint8_t data[8];

    // Knock % cyl #1
    // units: %
    uint16_t knock_cyl1_raw() const   {return data[0];}
    uint16_t knock_cyl1_whole() const {return data[0] * 4 / 10;}
    uint16_t knock_cyl1_frac() const  {return data[0] * 4 / 10;}
    float    knock_cyl1_flt() const   {return (float)(data[0]) * 4 / 10;}

    // Knock % cyl #2
    // units: %
    uint16_t knock_cyl2_raw() const   {return data[1];}
    uint16_t knock_cyl2_whole() const {return data[1] * 4 / 10;}
    uint16_t knock_cyl2_frac() const  {return data[1] * 4 / 10;}
    float    knock_cyl2_flt() const   {return (float)(data[1]) * 4 / 10;}

    // Knock % cyl #3
    // units: %
    uint16_t knock_cyl3_raw() const   {return data[2];}
    uint16_t knock_cyl3_whole() const {return data[2] * 4 / 10;}
    uint16_t knock_cyl3_frac() const  {return data[2] * 4 / 10;}
    float    knock_cyl3_flt() const   {return (float)(data[2]) * 4 / 10;}

    // Knock % cyl #4
    // units: %
    uint16_t knock_cyl4_raw() const   {return data[3];}
    uint16_t knock_cyl4_whole() const {return data[3] * 4 / 10;}
    uint16_t knock_cyl4_frac() const  {return data[3] * 4 / 10;}
    float    knock_cyl4_flt() const   {return (float)(data[3]) * 4 / 10;}

    // Knock % cyl #5
    // units: %
    uint16_t knock_cyl5_raw() const   {return data[4];}
    uint16_t knock_cyl5_whole() const {return data[4] * 4 / 10;}
    uint16_t knock_cyl5_frac() const  {return data[4] * 4 / 10;}
    float    knock_cyl5_flt() const   {return (float)(data[4]) * 4 / 10;}

    // Knock % cyl #6
    // units: %
    uint16_t knock_cyl6_raw() const   {return data[5];}
    uint16_t knock_cyl6_whole() const {return data[5] * 4 / 10;}
    uint16_t knock_cyl6_frac() const  {return data[5] * 4 / 10;}
    float    knock_cyl6_flt() const   {return (float)(data[5]) * 4 / 10;}

    // Knock % cyl #7
    // units: %
    uint16_t knock_cyl7_raw() const   {return data[6];}
    uint16_t knock_cyl7_whole() const {return data[6] * 4 / 10;}
    uint16_t knock_cyl7_frac() const  {return data[6] * 4 / 10;}
    float    knock_cyl7_flt() const   {return (float)(data[6]) * 4 / 10;}

    // Knock % cyl #8
    // units: %
    uint16_t knock_cyl8_raw() const   {return data[7];}
    uint16_t knock_cyl8_whole() const {return data[7] * 4 / 10;}
    uint16_t knock_cyl8_frac() const  {return data[7] * 4 / 10;}
    float    knock_cyl8_flt() const   {return (float)(data[7]) * 4 / 10;}
  };

  struct RtMsg49_t
  {
    uint8_t data[8];

    // Knock % cyl #9
    // units: %
    uint16_t knock_cyl9_raw() const   {return data[0];}
    uint16_t knock_cyl9_whole() const {return data[0] * 4 / 10;}
    uint16_t knock_cyl9_frac() const  {return data[0] * 4 / 10;}
    float    knock_cyl9_flt() const   {return (float)(data[0]) * 4 / 10;}

    // Knock % cyl #10
    // units: %
    uint16_t knock_cyl10_raw() const   {return data[1];}
    uint16_t knock_cyl10_whole() const {return data[1] * 4 / 10;}
    uint16_t knock_cyl10_frac() const  {return data[1] * 4 / 10;}
    float    knock_cyl10_flt() const   {return (float)(data[1]) * 4 / 10;}

    // Knock % cyl #11
    // units: %
    uint16_t knock_cyl11_raw() const   {return data[2];}
    uint16_t knock_cyl11_whole() const {return data[2] * 4 / 10;}
    uint16_t knock_cyl11_frac() const  {return data[2] * 4 / 10;}
    float    knock_cyl11_flt() const   {return (float)(data[2]) * 4 / 10;}

    // Knock % cyl #12
    // units: %
    uint16_t knock_cyl12_raw() const   {return data[3];}
    uint16_t knock_cyl12_whole() const {return data[3] * 4 / 10;}
    uint16_t knock_cyl12_frac() const  {return data[3] * 4 / 10;}
    float    knock_cyl12_flt() const   {return (float)(data[3]) * 4 / 10;}

    // Knock % cyl #13
    // units: %
    uint16_t knock_cyl13_raw() const   {return data[4];}
    uint16_t knock_cyl13_whole() const {return data[4] * 4 / 10;}
    uint16_t knock_cyl13_frac() const  {return data[4] * 4 / 10;}
    float    knock_cyl13_flt() const   {return (float)(data[4]) * 4 / 10;}

    // Knock % cyl #14
    // units: %
    uint16_t knock_cyl14_raw() const   {return data[5];}
    uint16_t knock_cyl14_whole() const {return data[5] * 4 / 10;}
    uint16_t knock_cyl14_frac() const  {return data[5] * 4 / 10;}
    float    knock_cyl14_flt() const   {return (float)(data[5]) * 4 / 10;}

    // Knock % cyl #15
    // units: %
    uint16_t knock_cyl15_raw() const   {return data[6];}
    uint16_t knock_cyl15_whole() const {return data[6] * 4 / 10;}
    uint16_t knock_cyl15_frac() const  {return data[6] * 4 / 10;}
    float    knock_cyl15_flt() const   {return (float)(data[6]) * 4 / 10;}

    // Knock % cyl #16
    // units: %
    uint16_t knock_cyl16_raw() const   {return data[7];}
    uint16_t knock_cyl16_whole() const {return data[7] * 4 / 10;}
    uint16_t knock_cyl16_frac() const  {return data[7] * 4 / 10;}
    float    knock_cyl16_flt() const   {return (float)(data[7]) * 4 / 10;}
  };

  struct RtMsg50_t
  {
    uint8_t data[8];

    // MAPdot based accel
    // units: %
    uint16_t map_accel_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t map_accel_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t map_accel_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    map_accel_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Total accel
    // units: %
    uint16_t total_accel_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t total_accel_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t total_accel_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    total_accel_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Timer for timed-launch retard
    // units: s
    uint16_t launch_timer_raw() const   {return MSG_GET_U16(data,5);}
    uint16_t launch_timer_whole() const {return MSG_GET_U16(data,5) * 1 / 1000;}
    uint16_t launch_timer_frac() const  {return MSG_GET_U16(data,5) * 1 / 1000;}
    float    launch_timer_flt() const   {return (float)(MSG_GET_U16(data,5)) * 1 / 1000;}

    // Launch retard
    // units: deg
    uint16_t launch_retard_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t launch_retard_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t launch_retard_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    launch_retard_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg51_t
  {
    uint8_t data[8];

    // CPU portA bitfield
    uint16_t porta_raw() const   {return data[0];}
    uint16_t porta_whole() const {return data[0] * 1 / 1;}
    uint16_t porta_frac() const  {return data[0] * 1 / 1;}
    float    porta_flt() const   {return (float)(data[0]) * 1 / 1;}

    // CPU portB bitfield
    uint16_t portb_raw() const   {return data[1];}
    uint16_t portb_whole() const {return data[1] * 1 / 1;}
    uint16_t portb_frac() const  {return data[1] * 1 / 1;}
    float    portb_flt() const   {return (float)(data[1]) * 1 / 1;}

    // CPU portE/portH bitfield
    uint16_t porteh_raw() const   {return data[2];}
    uint16_t porteh_whole() const {return data[2] * 1 / 1;}
    uint16_t porteh_frac() const  {return data[2] * 1 / 1;}
    float    porteh_flt() const   {return (float)(data[2]) * 1 / 1;}

    // CPU portK bitfield
    uint16_t portk_raw() const   {return data[3];}
    uint16_t portk_whole() const {return data[3] * 1 / 1;}
    uint16_t portk_frac() const  {return data[3] * 1 / 1;}
    float    portk_flt() const   {return (float)(data[3]) * 1 / 1;}

    // CPU portM/portJ bitfield
    uint16_t portmj_raw() const   {return data[4];}
    uint16_t portmj_whole() const {return data[4] * 1 / 1;}
    uint16_t portmj_frac() const  {return data[4] * 1 / 1;}
    float    portmj_flt() const   {return (float)(data[4]) * 1 / 1;}

    // CPU portP bitfield
    uint16_t portp_raw() const   {return data[5];}
    uint16_t portp_whole() const {return data[5] * 1 / 1;}
    uint16_t portp_frac() const  {return data[5] * 1 / 1;}
    float    portp_flt() const   {return (float)(data[5]) * 1 / 1;}

    // CPU portT bitfield
    uint16_t portt_raw() const   {return data[6];}
    uint16_t portt_whole() const {return data[6] * 1 / 1;}
    uint16_t portt_frac() const  {return data[6] * 1 / 1;}
    float    portt_flt() const   {return (float)(data[6]) * 1 / 1;}

    // CEL error code
    uint16_t cel_errorcode_raw() const   {return data[7];}
    uint16_t cel_errorcode_whole() const {return data[7] * 1 / 1;}
    uint16_t cel_errorcode_frac() const  {return data[7] * 1 / 1;}
    float    cel_errorcode_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg52_t
  {
    uint8_t data[8];

    // CAN input 1 bitfield (CAN port 1 on MS2)
    uint16_t canin1_raw() const   {return data[0];}
    uint16_t canin1_whole() const {return data[0] * 1 / 1;}
    uint16_t canin1_frac() const  {return data[0] * 1 / 1;}
    float    canin1_flt() const   {return (float)(data[0]) * 1 / 1;}

    // CAN input 2 bitfield (CAN port 2 on MS2)
    uint16_t canin2_raw() const   {return data[1];}
    uint16_t canin2_whole() const {return data[1] * 1 / 1;}
    uint16_t canin2_frac() const  {return data[1] * 1 / 1;}
    float    canin2_flt() const   {return (float)(data[1]) * 1 / 1;}

    // CAN output 1 bitfield (CAN port 3 on MS2)
    uint16_t canout_raw() const   {return data[2];}
    uint16_t canout_whole() const {return data[2] * 1 / 1;}
    uint16_t canout_frac() const  {return data[2] * 1 / 1;}
    float    canout_flt() const   {return (float)(data[2]) * 1 / 1;}

    // Knock retard
    // units: deg
    uint16_t knk_rtd_raw() const   {return data[3];}
    uint16_t knk_rtd_whole() const {return data[3] * 1 / 10;}
    uint16_t knk_rtd_frac() const  {return data[3] * 1 / 10;}
    float    knk_rtd_flt() const   {return (float)(data[3]) * 1 / 10;}

    // Average fuel flow
    // units: cc/min
    uint16_t fuelflow_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t fuelflow_whole() const {return MSG_GET_U16(data,4) * 1 / 1;}
    uint16_t fuelflow_frac() const  {return MSG_GET_U16(data,4) * 1 / 1;}
    float    fuelflow_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 1;}

    // Average fuel consumption
    // units: l/km
    uint16_t fuelcons_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t fuelcons_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t fuelcons_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    fuelcons_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg53_t
  {
    uint8_t data[8];

    // Fuel pressure 1
    // units: kPa
    uint16_t fuel_press1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t fuel_press1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t fuel_press1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    fuel_press1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Fuel pressure 2
    // units: kPa
    uint16_t fuel_press2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t fuel_press2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t fuel_press2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    fuel_press2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Fuel temperature 1
    // units: deg F
    uint16_t fuel_temp1_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t fuel_temp1_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t fuel_temp1_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    fuel_temp1_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Fuel temperature 2
    // units: deg F
    uint16_t fuel_temp2_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t fuel_temp2_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t fuel_temp2_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    fuel_temp2_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg54_t
  {
    uint8_t data[8];

    // Battery current (alternator system)
    // units: A
    uint16_t batt_cur_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t batt_cur_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t batt_cur_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    batt_cur_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // CEL status bitfield
    uint16_t cel_status_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t cel_status_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t cel_status_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    cel_status_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // Fuel pump output duty
    // units: %
    uint16_t fp_duty_raw() const   {return data[4];}
    uint16_t fp_duty_whole() const {return data[4] * 392 / 1000;}
    uint16_t fp_duty_frac() const  {return data[4] * 392 / 1000;}
    float    fp_duty_flt() const   {return (float)(data[4]) * 392 / 1000;}

    // Alternator field output duty
    // units: %
    uint16_t alt_duty_raw() const   {return data[5];}
    uint16_t alt_duty_whole() const {return data[5] * 1 / 1;}
    uint16_t alt_duty_frac() const  {return data[5] * 1 / 1;}
    float    alt_duty_flt() const   {return (float)(data[5]) * 1 / 1;}

    // Alternator measured load-sense duty
    // units: %
    uint16_t load_duty_raw() const   {return data[6];}
    uint16_t load_duty_whole() const {return data[6] * 1 / 1;}
    uint16_t load_duty_frac() const  {return data[6] * 1 / 1;}
    float    load_duty_flt() const   {return (float)(data[6]) * 1 / 1;}

    // Alternator target voltage
    // units: V
    uint16_t alt_targv_raw() const   {return data[7];}
    uint16_t alt_targv_whole() const {return data[7] * 1 / 10;}
    uint16_t alt_targv_frac() const  {return data[7] * 1 / 10;}
    float    alt_targv_flt() const   {return (float)(data[7]) * 1 / 10;}
  };

  struct RtMsg55_t
  {
    uint8_t data[8];

    // Main code loop execution time
    // units: us
    uint16_t looptime_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t looptime_whole() const {return MSG_GET_U16(data,0) * 1 / 1;}
    uint16_t looptime_frac() const  {return MSG_GET_U16(data,0) * 1 / 1;}
    float    looptime_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 1;}

    // Fuel temperature correction
    // units: %
    uint16_t fueltemp_cor_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t fueltemp_cor_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t fueltemp_cor_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    fueltemp_cor_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Fuel pressure correction
    // units: %
    uint16_t fuelpress_cor_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t fuelpress_cor_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t fuelpress_cor_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    fuelpress_cor_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Long term trim correction
    // units: %
    uint16_t ltt_cor_raw() const   {return data[6];}
    uint16_t ltt_cor_whole() const {return data[6] * 1 / 10;}
    uint16_t ltt_cor_frac() const  {return data[6] * 1 / 10;}
    float    ltt_cor_flt() const   {return (float)(data[6]) * 1 / 10;}

    // Unused
    // units: 
    uint16_t sp1_raw() const   {return data[7];}
    uint16_t sp1_whole() const {return data[7] * 1 / 1;}
    uint16_t sp1_frac() const  {return data[7] * 1 / 1;}
    float    sp1_flt() const   {return (float)(data[7]) * 1 / 1;}
  };

  struct RtMsg56_t
  {
    uint8_t data[8];

    // Traction control retard
    // units: deg
    uint16_t tc_retard_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t tc_retard_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t tc_retard_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    tc_retard_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // CEL retard
    // units: deg
    uint16_t cel_retard_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t cel_retard_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t cel_retard_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    cel_retard_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Fuel-cut (overrun) retard
    // units: deg
    uint16_t fc_retard_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t fc_retard_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t fc_retard_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    fc_retard_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // ALS added fuel
    // units: ms
    uint16_t als_addfuel_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t als_addfuel_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t als_addfuel_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    als_addfuel_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg57_t
  {
    uint8_t data[8];

    // Base timing from tables
    // units: deg
    uint16_t base_advance_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t base_advance_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t base_advance_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    base_advance_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Idle correction advance
    // units: deg
    uint16_t idle_cor_advance_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t idle_cor_advance_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t idle_cor_advance_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    idle_cor_advance_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // MAT retard
    // units: deg
    uint16_t mat_retard_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t mat_retard_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t mat_retard_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    mat_retard_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Flex advance
    // units: deg
    uint16_t flex_advance_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t flex_advance_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t flex_advance_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    flex_advance_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg58_t
  {
    uint8_t data[8];

    // Timing lookup from table 1
    // units: deg
    uint16_t adv1_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t adv1_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t adv1_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    adv1_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // Timing lookup from table 2
    // units: deg
    uint16_t adv2_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t adv2_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t adv2_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    adv2_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Timing lookup from table 3
    // units: deg
    uint16_t adv3_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t adv3_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t adv3_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    adv3_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Timing lookup from table 4
    // units: deg
    uint16_t adv4_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t adv4_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t adv4_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    adv4_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

  struct RtMsg59_t
  {
    uint8_t data[8];

    // Revlimiter 'soft' retard
    // units: deg
    uint16_t revlim_retard_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t revlim_retard_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t revlim_retard_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    revlim_retard_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // ALS timing change
    // units: deg
    uint16_t als_timing_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t als_timing_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t als_timing_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    als_timing_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // External advance (e.g. trans)
    // units: deg
    uint16_t ext_advance_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t ext_advance_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t ext_advance_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    ext_advance_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // Injector deadtime in use (#1)
    // units: ms
    uint16_t deadtime1_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t deadtime1_whole() const {return MSG_GET_U16(data,6) * 1 / 1000;}
    uint16_t deadtime1_frac() const  {return MSG_GET_U16(data,6) * 1 / 1000;}
    float    deadtime1_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1000;}
  };

  struct RtMsg60_t
  {
    uint8_t data[8];

    // Launch control timing
    // units: deg
    uint16_t launch_timing_raw() const   {return MSG_GET_U16(data,0);}
    uint16_t launch_timing_whole() const {return MSG_GET_U16(data,0) * 1 / 10;}
    uint16_t launch_timing_frac() const  {return MSG_GET_U16(data,0) * 1 / 10;}
    float    launch_timing_flt() const   {return (float)(MSG_GET_U16(data,0)) * 1 / 10;}

    // 3-step timing
    // units: deg
    uint16_t step3_timing_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t step3_timing_whole() const {return MSG_GET_U16(data,2) * 1 / 10;}
    uint16_t step3_timing_frac() const  {return MSG_GET_U16(data,2) * 1 / 10;}
    float    step3_timing_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 10;}

    // Wheel-speed based launch retard
    // units: deg
    uint16_t vsslaunch_retard_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t vsslaunch_retard_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t vsslaunch_retard_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    vsslaunch_retard_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // CEL status 2
    uint16_t cel_status2_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t cel_status2_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t cel_status2_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    cel_status2_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg61_t
  {
    uint8_t data[8];

    // External GPS latitude deg
    // units: deg
    uint16_t gps_latdeg_raw() const   {return data[0];}
    uint16_t gps_latdeg_whole() const {return data[0] * 1 / 1;}
    uint16_t gps_latdeg_frac() const  {return data[0] * 1 / 1;}
    float    gps_latdeg_flt() const   {return (float)(data[0]) * 1 / 1;}

    // GPS latitude minutes
    // units: min
    uint16_t gps_latmin_raw() const   {return data[1];}
    uint16_t gps_latmin_whole() const {return data[1] * 1 / 1;}
    uint16_t gps_latmin_frac() const  {return data[1] * 1 / 1;}
    float    gps_latmin_flt() const   {return (float)(data[1]) * 1 / 1;}

    // GPS latitude milli-minutes
    // units: mmin
    uint16_t gps_latmmin_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t gps_latmmin_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t gps_latmmin_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    gps_latmmin_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // GPS longitude degree
    // units: deg
    uint16_t gps_londeg_raw() const   {return data[4];}
    uint16_t gps_londeg_whole() const {return data[4] * 1 / 1;}
    uint16_t gps_londeg_frac() const  {return data[4] * 1 / 1;}
    float    gps_londeg_flt() const   {return (float)(data[4]) * 1 / 1;}

    // GPS longitude minute
    // units: min
    uint16_t gps_lonmin_raw() const   {return data[5];}
    uint16_t gps_lonmin_whole() const {return data[5] * 1 / 1;}
    uint16_t gps_lonmin_frac() const  {return data[5] * 1 / 1;}
    float    gps_lonmin_flt() const   {return (float)(data[5]) * 1 / 1;}

    // GPS longitude milli-minutes
    // units: mmin
    uint16_t gps_lonmmin_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t gps_lonmmin_whole() const {return MSG_GET_U16(data,6) * 1 / 1;}
    uint16_t gps_lonmmin_frac() const  {return MSG_GET_U16(data,6) * 1 / 1;}
    float    gps_lonmmin_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 1;}
  };

  struct RtMsg62_t
  {
    uint8_t data[8];

    // GPS status byte (bit 0 = E/W)
    uint16_t gps_outstatus_raw() const   {return data[0];}
    uint16_t gps_outstatus_whole() const {return data[0] * 1 / 1;}
    uint16_t gps_outstatus_frac() const  {return data[0] * 1 / 1;}
    float    gps_outstatus_flt() const   {return (float)(data[0]) * 1 / 1;}

    // GPS altitude km
    // units: km
    uint16_t gps_altk_raw() const   {return data[1];}
    uint16_t gps_altk_whole() const {return data[1] * 1 / 1;}
    uint16_t gps_altk_frac() const  {return data[1] * 1 / 1;}
    float    gps_altk_flt() const   {return (float)(data[1]) * 1 / 1;}

    // GPS altitude m
    // units: m
    uint16_t gps_altm_raw() const   {return MSG_GET_U16(data,2);}
    uint16_t gps_altm_whole() const {return MSG_GET_U16(data,2) * 1 / 1;}
    uint16_t gps_altm_frac() const  {return MSG_GET_U16(data,2) * 1 / 1;}
    float    gps_altm_flt() const   {return (float)(MSG_GET_U16(data,2)) * 1 / 1;}

    // GPS speed
    // units: ms-1
    uint16_t gps_speed_raw() const   {return MSG_GET_U16(data,4);}
    uint16_t gps_speed_whole() const {return MSG_GET_U16(data,4) * 1 / 10;}
    uint16_t gps_speed_frac() const  {return MSG_GET_U16(data,4) * 1 / 10;}
    float    gps_speed_flt() const   {return (float)(MSG_GET_U16(data,4)) * 1 / 10;}

    // GPS course
    // units: deg
    uint16_t gps_course_raw() const   {return MSG_GET_U16(data,6);}
    uint16_t gps_course_whole() const {return MSG_GET_U16(data,6) * 1 / 10;}
    uint16_t gps_course_frac() const  {return MSG_GET_U16(data,6) * 1 / 10;}
    float    gps_course_flt() const   {return (float)(MSG_GET_U16(data,6)) * 1 / 10;}
  };

}
