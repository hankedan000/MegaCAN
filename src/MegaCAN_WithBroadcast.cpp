#include "MegaCAN_WithBroadcast.h"

namespace MegaCAN
{

DeviceWithBroadcast::DeviceWithBroadcast(
		uint8_t cs,
		uint8_t myId,
		uint8_t intPin,
		CAN_Msg *buff,
		uint8_t buffSize)
	: Device(cs,myId,intPin,buff,buffSize)
{
}

DeviceWithBroadcast::~DeviceWithBroadcast()
{
}

void
DeviceWithBroadcast::handleStandard(
		const uint32_t id,
		const uint8_t length,
		uint8_t* data)
{
	DEBUG("handleStandard");

	switch (id)
	{
		case MSG00_ID:
		{
			MSG00_t* msg = reinterpret_cast<MSG00_t*>(data);
			eng_.rpm = bswap16(msg->rpm);
			eng_.pw = bswap16(msg->pw1);

			DEBUG(
					"time = %d; pw1 = %d; pw2 = %d; rpm = %d",
					bswap16(msg->seconds),
					bswap16(msg->pw1),
					bswap16(msg->pw2),
					bswap16(msg->rpm));
			break;
		}
		case MSG02_ID:
		{
			MSG02_t* msg = reinterpret_cast<MSG02_t*>(data);
			eng_.clt = bswap16(msg->clt);
			eng_.mat = bswap16(msg->mat);

			DEBUG(
					"baro = %d.%d; map = %d.%d; clt_F = %d.%d",
					bswap16(msg->baro)/10,abs(bswap16(msg->baro)%10),
					bswap16(msg->map)/10,abs(bswap16(msg->map)%10),
					bswap16(msg->clt)/10,abs(bswap16(msg->clt)%10));
			break;
		}
		case MSG03_ID:
		{
			MSG03_t* msg = reinterpret_cast<MSG03_t*>(data);
			eng_.batt = bswap16(msg->batt);

			DEBUG(
					"tps = %d.%d; batt = %d.%d",
					bswap16(msg->tps)/10,abs(bswap16(msg->tps)%10),
					bswap16(msg->batt)/10,bswap16(msg->batt)%10);
			break;
		}
		case MSG10_ID:
		{
			MSG10_t* msg = reinterpret_cast<MSG10_t*>(data);

			DEBUG(
					"status2 = 0x%d0.2x",
					msg->status2);
			break;
		}
		default:
		{
			WARN("Unsupported standard MSG%d_ID",id);
			break;
		}
	}// switch
}

}// namespace - MegaCAN