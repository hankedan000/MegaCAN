#include "RealtimeDataListener.h"

RealtimeDataListener::RealtimeDataListener(
  uint8_t cs,
  uint8_t myId,
  uint8_t intPin,
  MegaCAN::CAN_Msg *buff,
  uint8_t buffSize)
 : MegaCAN::Device(cs,myId,intPin,buff,buffSize)
{
}

void
RealtimeDataListener::getOptions(
  struct MegaCAN::Options *opts)
{
  opts->handleStandardMsgsImmediately = true;
}

void
RealtimeDataListener::applyCanFilters(
  MCP_CAN *can)
{
  // filter Megasquirt broadcast frames into RXB0
  can->init_Mask(0,0,0x00000000);
  can->init_Filt(0,0,0x00000000);
  can->init_Filt(1,0,0x00000000);

  // filter Megasquirt broadcast frames into RXB1
  can->init_Mask(1,0,0x00000000);
  can->init_Filt(2,0,0x00000000);
  can->init_Filt(3,0,0x00000000);
  can->init_Filt(4,0,0x00000000);
  can->init_Filt(5,0,0x00000000);
}

#define cpymsg(msgDataField, fromBuff, fromSize) memcpy(msgDataField, fromBuff, min(sizeof(msgDataField), fromSize))

void
RealtimeDataListener::handleStandard(
    const uint32_t id,
    const uint8_t length,
    uint8_t* data)
{
  const uint32_t msgId = id - 1520;

  switch (msgId)
  {
    case 0:
      cpymsg(data_.m0.data, data, length);
      break;
    case 1:
      cpymsg(data_.m1.data, data, length);
      break;
    case 2:
      cpymsg(data_.m2.data, data, length);
      break;
    case 3:
      cpymsg(data_.m3.data, data, length);
      break;
    case 4:
      cpymsg(data_.m4.data, data, length);
      break;
    case 5:
      cpymsg(data_.m5.data, data, length);
      break;
    case 6:
      cpymsg(data_.m6.data, data, length);
      break;
    case 7:
      cpymsg(data_.m7.data, data, length);
      break;
    case 8:
      cpymsg(data_.m8.data, data, length);
      break;
    case 9:
      cpymsg(data_.m9.data, data, length);
      break;
    case 10:
      cpymsg(data_.m10.data, data, length);
      break;
    case 11:
      cpymsg(data_.m11.data, data, length);
      break;
    case 12:
      cpymsg(data_.m12.data, data, length);
      break;
    case 13:
      cpymsg(data_.m13.data, data, length);
      break;
    case 14:
      cpymsg(data_.m14.data, data, length);
      break;
    case 15:
      cpymsg(data_.m15.data, data, length);
      break;
    case 16:
      cpymsg(data_.m16.data, data, length);
      break;
    case 17:
      cpymsg(data_.m17.data, data, length);
      break;
    case 18:
      cpymsg(data_.m18.data, data, length);
      break;
    case 19:
      cpymsg(data_.m19.data, data, length);
      break;
    case 20:
      cpymsg(data_.m20.data, data, length);
      break;
    case 21:
      cpymsg(data_.m21.data, data, length);
      break;
    case 22:
      cpymsg(data_.m22.data, data, length);
      break;
    case 23:
      cpymsg(data_.m23.data, data, length);
      break;
    case 24:
      cpymsg(data_.m24.data, data, length);
      break;
    case 25:
      cpymsg(data_.m25.data, data, length);
      break;
    case 26:
      cpymsg(data_.m26.data, data, length);
      break;
    case 27:
      cpymsg(data_.m27.data, data, length);
      break;
    case 28:
      cpymsg(data_.m28.data, data, length);
      break;
    case 29:
      cpymsg(data_.m29.data, data, length);
      break;
    case 30:
      cpymsg(data_.m30.data, data, length);
      break;
    case 31:
      cpymsg(data_.m31.data, data, length);
      break;
    case 32:
      cpymsg(data_.m32.data, data, length);
      break;
    case 33:
      cpymsg(data_.m33.data, data, length);
      break;
    case 34:
      cpymsg(data_.m34.data, data, length);
      break;
    case 35:
      cpymsg(data_.m35.data, data, length);
      break;
    case 36:
      cpymsg(data_.m36.data, data, length);
      break;
    case 37:
      cpymsg(data_.m37.data, data, length);
      break;
    case 38:
      cpymsg(data_.m38.data, data, length);
      break;
    case 39:
      cpymsg(data_.m39.data, data, length);
      break;
    case 40:
      cpymsg(data_.m40.data, data, length);
      break;
    case 41:
      cpymsg(data_.m41.data, data, length);
      break;
    case 42:
      cpymsg(data_.m42.data, data, length);
      break;
    case 43:
      cpymsg(data_.m43.data, data, length);
      break;
    case 44:
      cpymsg(data_.m44.data, data, length);
      break;
    case 45:
      cpymsg(data_.m45.data, data, length);
      break;
    case 46:
      cpymsg(data_.m46.data, data, length);
      break;
    case 47:
      cpymsg(data_.m47.data, data, length);
      break;
    case 48:
      cpymsg(data_.m48.data, data, length);
      break;
    case 49:
      cpymsg(data_.m49.data, data, length);
      break;
    case 50:
      cpymsg(data_.m50.data, data, length);
      break;
    case 51:
      cpymsg(data_.m51.data, data, length);
      break;
    case 52:
      cpymsg(data_.m52.data, data, length);
      break;
    case 53:
      cpymsg(data_.m53.data, data, length);
      break;
    case 54:
      cpymsg(data_.m54.data, data, length);
      break;
    case 55:
      cpymsg(data_.m55.data, data, length);
      break;
    case 56:
      cpymsg(data_.m56.data, data, length);
      break;
    case 57:
      cpymsg(data_.m57.data, data, length);
      break;
    case 58:
      cpymsg(data_.m58.data, data, length);
      break;
    case 59:
      cpymsg(data_.m59.data, data, length);
      break;
    case 60:
      cpymsg(data_.m60.data, data, length);
      break;
    case 61:
      cpymsg(data_.m61.data, data, length);
      break;
    case 62:
      cpymsg(data_.m62.data, data, length);
      break;
    default:
      // ignore data
      break;
  }
}