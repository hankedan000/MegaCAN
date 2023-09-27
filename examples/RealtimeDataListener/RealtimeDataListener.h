#include "MSG_defn.h"
#pragma once

#include <MegaCAN_Device.h>

struct RT_Data
{
  MSG00_t m0;
  MSG02_t m2;
  MSG03_t m3;
  MSG10_t m10;
};

// A class that listens for Megasaquirt realtime data messages. It stores
// and provides access to the most recent engine data received from the bus.
class RealtimeDataListener : public MegaCAN::Device
{
public:
	RealtimeDataListener(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			MegaCAN::CAN_Msg *buff,
			uint8_t buffSize);

  const RT_Data &
  data() const
  {
    return data_;
  }

protected:
  // override so we can mark option to handle standard msgs immediately
	virtual void
	getOptions(
		struct MegaCAN::Options *opts) override;
  
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
			const uint32_t id,
			const uint8_t length,
			uint8_t *data) override;

private:
  RT_Data data_;

};