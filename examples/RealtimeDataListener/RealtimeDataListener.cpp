#include "MegaCAN_Device.h"
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
  MegaCAN::Options *opts)
{
  opts->handleStandardMsgsImmediately = true;
}

#define cpymsg(msgDataField, fromBuff, fromSize) memcpy(msgDataField, fromBuff, min(sizeof(msgDataField), fromSize))

void
RealtimeDataListener::handleStandard(
		const uint32_t id,
		const uint8_t length,
		uint8_t* data)
{
	DEBUG("handleStandard");
  const uint32_t msgId = id - 1520;

	switch (msgId)
	{
		case 0:
      cpymsg(data_.m0.data, data, length);
			break;
		case 2:
      cpymsg(data_.m2.data, data, length);
			break;
		case 3:
      cpymsg(data_.m3.data, data, length);
			break;
		case 10:
      cpymsg(data_.m10.data, data, length);
			break;
		default:
      // ignore data
			break;
	}
}