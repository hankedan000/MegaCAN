#ifndef MSG_DEFN_H_
#define MSG_DEFN_H_

#include <stdint.h>

#define bswap16(x) __builtin_bswap16(x)

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

#define MSG_BASE  1520

#define MSG00_ID  MSG_BASE+0
struct MSG00_t{
  uint8_t data[8];

  uint16_t seconds() const {return bswap16(*(const uint16_t *)(data+0));}

  // returns pulse width in milliseconds
  uint16_t pw1() const {return bswap16(*(const uint16_t *)(data+2));}

  // returned pulse width in microseconds
  uint16_t pw2() const {return bswap16(*(const uint16_t *)(data+4));}

  uint16_t rpm() const {return bswap16(*(const uint16_t *)(data+6));}
};

#define MSG02_ID  MSG_BASE+2
#define MSQ_BARO_DIV 10
#define MSQ_MAP_DIV  10
#define MSQ_MAT_DIV  10
#define MSQ_CLT_DIV  10
struct MSG02_t{
  uint8_t data[8];

  // returned barometric pressure in kPa
  uint16_t baro() const {return bswap16(*(const uint16_t *)(data+0))/MSQ_BARO_DIV;}

  // returned manifold pressure in kPa
  uint16_t map() const {return bswap16(*(const uint16_t *)(data+2))/MSQ_MAP_DIV;}

  // returned manifold air temp in degrees fahrenheit
  uint16_t mat() const {return bswap16(*(const uint16_t *)(data+4))/MSQ_MAT_DIV;}

  // returned coolant temp in degrees fahrenheit
  uint16_t clt() const {return bswap16(*(const uint16_t *)(data+6))/MSQ_CLT_DIV;}
};

#define MSG03_ID  MSG_BASE+3
#define MSQ_TPS_DIV   10
#define MSQ_BATT_DIV  10
#define MSQ_AFR1_DIV  10
#define MSQ_AFR2_DIV  10
struct MSG03_t{
  uint8_t data[8];

  // returned throttle position in percent
  uint16_t tps() const {return bswap16(*(const uint16_t *)(data+0))/MSQ_TPS_DIV;}

  // returned battery voltage in volts
  uint16_t batt() const {return bswap16(*(const uint16_t *)(data+2))/MSQ_BATT_DIV;}

  // returned AFR bank 1
  uint16_t afr1() const {return bswap16(*(const uint16_t *)(data+4))/MSQ_AFR1_DIV;}

  // returned AFR bank 2
  uint16_t afr2() const {return bswap16(*(const uint16_t *)(data+6))/MSQ_AFR2_DIV;}
};

#define MSG10_ID  MSG_BASE+10
struct MSG10_t{
  uint8_t data[8];

  uint8_t status1() const {return data[0];}
  uint8_t status2() const {return data[1];}
  uint8_t status3() const {return data[2];}
  uint8_t status4() const {return data[3];}
  uint16_t status5() const {return bswap16(*(const uint16_t *)(data+4));}
  uint8_t status6() const {return data[6];}
  uint8_t status7() const {return data[7];}
};

#endif
