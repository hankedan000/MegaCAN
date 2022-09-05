#ifndef MEGA_CAN_WITH_BROADCAST_H_
#define MEGA_CAN_WITH_BROADCAST_H_

#include <Arduino.h>
#include "MegaCAN_Device.h"

namespace MegaCAN
{

class DeviceWithBroadcast : public MegaCAN::Device
{
public:
	DeviceWithBroadcast(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin);

	DeviceWithBroadcast() = delete;

	virtual
	~DeviceWithBroadcast();

protected:
	/**
	 * Called when a standard 11bit megasquirt broadcast frame is received.
	 * 
	 * @param[in] id
	 * The 11bit CAN identifier
	 * 
	 * @param[in] length
	 * The number of data bytes in the CAN frame
	 * 
	 * @param[in] data
	 * A pointer to the data segment of the CAN frame
	 */
	virtual void
	handleStandard(
			const uint32_t &id,
			const uint8_t &length,
			uint8_t *data) override;

private:
	struct EngineParams_T
	{
		int16_t rpm;
		int16_t mat;
		int16_t batt;
		int16_t clt;
		uint16_t pw;
	};

	EngineParams_T eng_;

};

}// namespace - MegaCAN

#endif
