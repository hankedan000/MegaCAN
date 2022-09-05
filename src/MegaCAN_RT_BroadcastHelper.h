#ifndef MEGA_CAN_RT_BROADCAST_HELPER_H_
#define MEGA_CAN_RT_BROADCAST_HELPER_H_

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

#define RT_BCAST_RATE_1HZ  0
#define RT_BCAST_RATE_2HZ  1
#define RT_BCAST_RATE_5HZ  2
#define RT_BCAST_RATE_10HZ 3
#define RT_BCAST_RATE_25HZ 4
#define RT_BCAST_RATE_50HZ 5

#define NUM_RT_BCAST_GROUP_MASKS 4

namespace MegaCAN
{

struct RT_Broadcast_T
{
	union Control_T
	{
		struct ControlBits_T
		{
			// set to 1 to enable real-time data broadcasting
			uint8_t enabled  : 1;
			uint8_t reserve0 : 3;
			uint8_t rate     : 3;
			uint8_t reserve1 : 1;
		} bits;
		uint8_t value;
	} ctrl;
	uint16_t baseId;
	uint8_t groupMasks[NUM_RT_BCAST_GROUP_MASKS];
};

class RT_BroadcastHelper
{
public:
	RT_BroadcastHelper();

	// call once at startup time to configure timer ISR
	void
	setup(
		Scheduler* ts,
		const uint16_t &rtBcastFlashOffset,
		void (*groupSendCallback)(uint16_t /*baseId*/, uint8_t /*group*/));

	// call this method as frequent as possible (ie. in main loop)
	void
	execute();

	void
	doBroadcast();

private:
	// flash offset where user maintains a RT_Broadcast_T structure
	uint16_t RT_BCAST_OFFSET_;

	// callback to user code when a broadcast group needs to sent
	void (*callback_)(uint16_t /*baseId*/, uint8_t /*group*/);

	// TaskSceduler instance
	Scheduler *ts_;

	// Task scheduled for when to send real-time data
	Task *task_;

	uint8_t prevRate_;

};

extern RT_BroadcastHelper RT_Bcast;

}// namespace - MegaCan

#endif