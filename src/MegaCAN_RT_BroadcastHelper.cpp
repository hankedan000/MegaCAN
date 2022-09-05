#include "MegaCAN_RT_BroadcastHelper.h"

#include "FlashUtils.h"
#include "logging.h"

void
__RT_BCastTaskCallback();

namespace MegaCAN
{

// instatiate the helper
RT_BroadcastHelper RT_Bcast;

RT_BroadcastHelper::RT_BroadcastHelper()
 : RT_BCAST_OFFSET_(0)
 , callback_(0)
 , ts_(nullptr)
 , task_(nullptr)
 , prevRate_(-1)
{
}

void
RT_BroadcastHelper::setup(
	Scheduler* ts,
	const uint16_t &rtBcastFlashOffset,
	void (*groupSendCallback)(uint16_t /*baseId*/, uint8_t /*group*/))
{
	RT_BCAST_OFFSET_ = rtBcastFlashOffset;
	callback_ = groupSendCallback;

	task_ = new Task(0, TASK_FOREVER, __RT_BCastTaskCallback, ts, false);
}

void
RT_BroadcastHelper::execute()
{
	RT_Broadcast_T::Control_T ctrl;
	ctrl.value = EEPROM.read(RT_BCAST_OFFSET_ + offsetof(RT_Broadcast_T, ctrl));

	if (ctrl.bits.rate != prevRate_)
	{
		switch (ctrl.bits.rate)
		{
			case RT_BCAST_RATE_1HZ:
				task_->setInterval((1000 / 1) * TASK_MILLISECOND);
				break;
			case RT_BCAST_RATE_2HZ:
				task_->setInterval((1000 / 2) * TASK_MILLISECOND);
				break;
			case RT_BCAST_RATE_5HZ:
				task_->setInterval((1000 / 5) * TASK_MILLISECOND);
				break;
			case RT_BCAST_RATE_10HZ:
				task_->setInterval((1000 / 10) * TASK_MILLISECOND);
				break;
			case RT_BCAST_RATE_25HZ:
				task_->setInterval((1000 / 25) * TASK_MILLISECOND);
				break;
			case RT_BCAST_RATE_50HZ:
				task_->setInterval((1000 / 50) * TASK_MILLISECOND);
				break;
			default:
				WARN("invalid rate %d",ctrl.bits.rate);
				break;
		}
	}
	prevRate_ = ctrl.bits.rate;

	// enable interrupt for OCR1A
	if (ctrl.bits.enabled) task_->enableIfNot();
	else                   task_->disable();
}

void
RT_BroadcastHelper::doBroadcast()
{
	if ( ! callback_)
	{
		return;
	}

	uint16_t baseId = EEPROM_GetBigU16(RT_BCAST_OFFSET_ + offsetof(RT_Broadcast_T, baseId));

	// iterate over data groups and send those that are enabled
	size_t groupMaskAddr = RT_BCAST_OFFSET_ + offsetof(RT_Broadcast_T, groupMasks);
	uint8_t groupMask = 0;
	for (uint8_t g=0; g<NUM_RT_BCAST_GROUP_MASKS; g++)
	{
		groupMask = EEPROM.read(groupMaskAddr++);
		for (uint8_t gg=0; gg<8; gg++)
		{
			if (groupMask & (1<<gg))
			{
				callback_(baseId, g*8+gg);
			}
		}
	}
}

}// namespace - MegaCAN

void
__RT_BCastTaskCallback()
{
	MegaCAN::RT_Bcast.doBroadcast();
}