#include "MegaCAN_Device.h"

#include "MegaCAN/Panic.h"

#define INC_ERROR_COUNTER(VAR) if(VAR!=0xFF){VAR++;}
#define IS_VISIBLE_ASCII(c) (c >= 32 && c <= 126)

#define LOG_CAN_TRAFFIC 0// set to 1 to log CAN traffic to UART

namespace MegaCAN
{

#if LOG_CAN_TRAFFIC
	// "0x12345678 | len 4 | 00 00 00 00 00 00 00 00 | ........ |"
	char __canMsgBuff[64];

	const char *
	fmtCAN_DebugStr(
		uint32_t id,
		uint8_t ext,
		uint8_t len,
		uint8_t *buf)
	{
		sprintf(
			__canMsgBuff,
			"0x%08lx | len %d | %02x %02x %02x %02x %02x %02x %02x %02x | %c%c%c%c%c%c%c%c |",
			id | (!!ext << 31),
			len,
			// hex dump portion
			(len >= 1 ? buf[0] : 0x00),
			(len >= 2 ? buf[1] : 0x00),
			(len >= 3 ? buf[2] : 0x00),
			(len >= 4 ? buf[3] : 0x00),
			(len >= 5 ? buf[4] : 0x00),
			(len >= 6 ? buf[5] : 0x00),
			(len >= 7 ? buf[6] : 0x00),
			(len == 8 ? buf[7] : 0x00),
			// ascii dump portion
			(len >= 1 && IS_VISIBLE_ASCII(buf[0]) ? buf[0] : '.'),
			(len >= 2 && IS_VISIBLE_ASCII(buf[1]) ? buf[1] : '.'),
			(len >= 3 && IS_VISIBLE_ASCII(buf[2]) ? buf[2] : '.'),
			(len >= 4 && IS_VISIBLE_ASCII(buf[3]) ? buf[3] : '.'),
			(len >= 5 && IS_VISIBLE_ASCII(buf[4]) ? buf[4] : '.'),
			(len >= 6 && IS_VISIBLE_ASCII(buf[5]) ? buf[5] : '.'),
			(len >= 7 && IS_VISIBLE_ASCII(buf[6]) ? buf[6] : '.'),
			(len == 8 && IS_VISIBLE_ASCII(buf[7]) ? buf[7] : '.'));
		return __canMsgBuff;
	}
#endif

Device::Device(
	const SharedPtr<HAL::CAN_Bus> & canBus,
	const uint8_t myMsqId)
	: canBus_(canBus)
	, myMsqId_(myMsqId)
	, canStatus_(0x0)
	, numSimReqDropsLeft_(0)
{
	if ( ! canBus_) {
		MC_PANIC("canBus can't be null");
	} else if (myMsqId > MAX_MSQ_ID) {
		MC_PANIC("myMsqId must be <= MAX_MSQ_ID");
	}
	resetErrorCounters();
	setupOptions();
}

void
Device::init()
{
	bool okay = true;

	// TODO move this logic out
	// Initialize MCP2515 with a baudrate of 500kb/s
	// if(okay && can_.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
	// {
	// 	MC_LOG_ERROR("MCP2515::begin() failed!");
	// 	okay = false;
	// }

	// /**
	//  * setup hardware-based CAN filters
	//  * 
	//  * The default will filter by the Megasquirt CAN device ID, but the
	//  * subclass override the method to provide more custom filters.
	//  */
	// applyCanFilters(&can_);

	// // set the mode to "NORMAL" (only mode we can RX & TX in)
	// if (okay && can_.setMode(MCP_NORMAL) != CAN_OK)
	// {
	// 	MC_LOG_ERROR("MCP2515::setMode() failed!");
	// 	okay = false;
	// }

	if (okay)
	{
		MC_LOG_INFO("MegaCAN initialized!");
		MC_LOG_INFO("%d Rx CAN_Msg buffer (%ldbytes)",
				queue_.capacity(),
				queue_.capacity() * sizeof(HAL::CAN_Msg));
	}
	else
	{
		MC_LOG_ERROR("MegaCAN initialization failed!");
	}
}

void
Device::interrupt()
{
	auto msg = queue_.getBackPtr();

	while (canBus_->readAny(*msg) == HAL::CAN_Bus::RetCode::OK)
	{
		// see if we should handle the broadcast messages immediately
		if (opts_.handleStandardMsgsImmediately && ! msg->id.isExt())
		{
			handleStandard(msg->id.getId(), msg->data);
		}
		else
		{
			if (queue_.isFull())
			{
				canStatus_ |= CAN_STATUS_RX_OVERFLOW;
				INC_ERROR_COUNTER(canSW_RxOverflowCount_);
			}
			else
			{
				queue_.push();
				msg = queue_.getBackPtr();
			}
		}
	}
}

void
Device::handle()
{
	while ( ! queue_.isEmpty())
	{
		const auto msg = queue_.getFrontPtr();
#if LOG_CAN_TRAFFIC
		MC_LOG_INFO("BUS >>> MCU %s", fmtCAN_DebugStr(msg->id,msg->ext,msg->len,msg->data.data()));
#endif

		if (msg->id.isExt())
		{
			const MS_HDR_t* hdr = reinterpret_cast<const MS_HDR_t*>(&msg->id);

			if (hdr->toId == myMsqId_)
			{
				handleExtended(hdr,msg->data);
			}
			else
			{
				MC_LOG_WARN("msg not meant for me!!!");
			}
		}
		else
		{
			handleStandard(msg->id.getId(), msg->data);
		}

		queue_.pop();
	}
}

//--------------------------------------------------------------------
// Protected Methods
//--------------------------------------------------------------------

void
Device::getOptions(
	struct Options *opts)
{
	// base impl leaves defaults
}

bool
Device::readFromTable(
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		HAL::CAN_DataBuffer & dataOut)
{
	MC_LOG_WARN("subclass should override readFromTable()");
	return false;
}

bool
Device::writeToTable(
		const uint8_t table,
		const uint16_t offset,
		const HAL::CAN_DataBuffer & data)
{
	MC_LOG_WARN("subclass should override writeToTable()");
	return false;
}


bool
Device::burnTable(
		const uint8_t table)
{
	MC_LOG_WARN("subclass should override burnTable()");
	return false;
}

// provide default implementation, but subclass should override
uint16_t
Device::tableBlockingFactor()
{
	static bool warnedOnce = false;
	if ( ! warnedOnce)
	{
		MC_LOG_WARN("using default table blocking factor");
		warnedOnce = true;
	}
	return 1;
}

// provide default implementation, but subclass should override
uint16_t
Device::writeBlockingFactor()
{
	static bool warnedOnce = false;
	if ( ! warnedOnce)
	{
		MC_LOG_WARN("using default write blocking factor");
		warnedOnce = true;
	}
	return 1;
}

void
Device::handleStandard(
		const uint32_t id,
		const HAL::CAN_DataBuffer & data)
{
	// base class doesn't provide broadcast support
	// overide this method, or use MegaCan_WithBroadcast
}

void
Device::simReqDrop(
	uint8_t numReqsToDrop)
{
	numSimReqDropsLeft_ = numReqsToDrop;
}

//--------------------------------------------------------------------
// Private Methods
//--------------------------------------------------------------------

void
Device::setupOptions()
{
	// setup default options
	opts_.handleStandardMsgsImmediately = false;
	getOptions(&opts_);// allow subclass to customize
}

void
Device::handleExtended(
		const MS_HDR_t *hdr,
		const HAL::CAN_DataBuffer & data)
{
	uint8_t table = getTable(hdr);
	MC_LOG_DEBUG(
			"handleExtended - "
			"table: %d; toId %d; fromId: %d; type: %d; offset: %d",
			table,
			hdr->toId,
			hdr->fromId,
			hdr->type,
			hdr->offset);

	switch(hdr->type)
	{
	case MSG_CMD:
		// TODO do something with return value
		writeToTable(table,hdr->offset,data);
		break;
	case MSG_REQ:
		if (numSimReqDropsLeft_ == 0)
		{
			handleRequest(hdr,data);
		}
		else
		{
			numSimReqDropsLeft_--;
		}
		break;
	case MSG_BURN:
	{
		bool burnOkay = burnTable(table);

		// submit burn acknowledge
		rspHdr_.toId = hdr->fromId;
		rspHdr_.fromId = hdr->toId;
		rspHdr_.type = MSG_XTND;
		setTable(&rspHdr_,0);// don't care
		rspHdr_.offset = 0;// don't care
		txMsg_.id.setId(true, rspHdr_.marshal());
		txMsg_.data.resize_nofill(2u);
		txMsg_.data[0] = MSG_BURNACK;
		txMsg_.data[1] = (burnOkay ? 1 : 0);

		if ( ! sendMsg(txMsg_))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
		break;
	}
	case MSG_XTND:
		handleExtendedMsg(hdr, data);
		break;
	default:
		MC_LOG_ERROR("Unimplemented MSG type: %d", hdr->type);
		break;
	}// switch
}

void
Device::handleRequest(
		const MS_HDR_t * hdr,
		const HAL::CAN_DataBuffer & reqData)
{
	bool okay = true;
	auto req = reinterpret_cast<const MSG_REQ_t*>(reqData.data());
	uint8_t reqTable = getTable(hdr);
	uint16_t rspOffset = getOffset(req);

	MC_LOG_DEBUG(
			"handleRequest - "
			"reqTable: %d; reqOffset: %d; resTable: %d; resLen: %d; resOffset: %d",
			reqTable,
			hdr->offset,
			req->rspTable,
			req->rspLength,
			rspOffset);

	txMsg_.data.resize_nofill(0);
	if (reqTable == TABLE_NO_SIG)
	{
		// Send the firmware signature

		if (hdr->offset == 0)
		{
			MC_LOG_DEBUG("CanID %d requested signature: '%s'",
					hdr->fromId,
					__MegaCAN_SerialSignature);
		}

		if ((hdr->offset + req->rspLength) > MAX_SIGNATURE_BYTES)
		{
			MC_LOG_ERROR("Requested too many bytes from signature!");
			okay = false;
		}
		else
		{
			// send the firmware signature. pad with trailing zeros.
			uint16_t offset = hdr->offset;
			for (uint8_t i=0; i<req->rspLength; i++, offset++)
			{
				if (offset < MAX_SIGNATURE_BYTES)
				{
					txMsg_.data.push_back(__MegaCAN_SerialRevision[offset]);
				}
				else
				{
					txMsg_.data.push_back('\0');
				}
			}
		}
	}
	else if (reqTable == TABLE_NO_REV)
	{
		// Send the firmware revision

		if (hdr->offset == 0)
		{
			MC_LOG_DEBUG("CanID %d requested revision: '%s'",
					hdr->fromId,
					__MegaCAN_SerialRevision);
		}

		// send the firmware revision. pad with trailing zeros.
		uint16_t offset = hdr->offset;
		for (uint8_t i=0; i<req->rspLength; i++, offset++)
		{
			if (offset < __MegaCAN_SerialRevisionLen)
			{
				txMsg_.data.push_back(__MegaCAN_SerialRevision[offset]);
			}
			else
			{
				txMsg_.data.push_back('\0');
			}
		}
	}
	else
	{
		okay = okay && readFromTable(
				reqTable,
				hdr->offset,
				req->rspLength,
				txMsg_.data);
	}

	rspHdr_.toId = hdr->fromId;
	rspHdr_.fromId = hdr->toId;
	rspHdr_.type = MSG_RSP;
	setTable(&rspHdr_,req->rspTable);
	rspHdr_.offset = rspOffset;
	txMsg_.id.setId(true, rspHdr_.marshal());

	if (okay)
	{
		// send MSG_RSP packet
		if ( ! sendMsg(txMsg_))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
	}
	else
	{
		// handle response, but send back zeros
		txMsg_.data.resize(req->rspLength, 0u);
		if ( ! sendMsg(txMsg_))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
	}
}

void
Device::handleExtendedMsg(
		const MS_HDR_t *hdr,
		const HAL::CAN_DataBuffer & data)
{
	uint8_t rspLength;

	if (data.size() < 1)
	{
		MC_LOG_ERROR("Invalid EXT MSG length %d", data.size());
		return;
	}

	switch(data[0])
	{

	case MSG_PROT:
	{
		if (data.size() == 4)
		{
			rspLength = GET_MSG_PROT_VARBYT(data);

			// populate response header
			rspHdr_.toId = hdr->fromId;
			rspHdr_.fromId = hdr->toId;
			rspHdr_.type = MSG_RSP;
			setTable(&rspHdr_,GET_MSG_PROT_MYVARBLK(data));
			rspHdr_.offset = GET_MSG_PROT_MYVAROFFSET(data);

			MC_LOG_DEBUG(
				"MSG_PROT: rspLength = %d, rspTable = %d, rspOffset = %d",
				rspLength,
				getTable(&rspHdr_),
				rspHdr_.offset);

			if (rspLength == 1)
			{
				// send serial version
				txMsg_.data.resize_nofill(1);
				txMsg_.data[0] = 2;// serial version
			}
			else if(rspLength == 5)
			{
				// send serial version with block factor
				txMsg_.data.resize_nofill(5);
				txMsg_.data[0] = 2;// serial version
				txMsg_.data[1] = tableBlockingFactor() >> 8;  // high byte
				txMsg_.data[2] = tableBlockingFactor() & 0xff;// low byte
				txMsg_.data[3] = writeBlockingFactor() >> 8;  // high byte
				txMsg_.data[4] = writeBlockingFactor() & 0xff;// low byte
			}
			else
			{
				MC_LOG_ERROR("Invalid MSG_PROT rspLength %d", rspLength);
				return;
			}

			// send protocol response
			txMsg_.id.setId(true, rspHdr_.marshal());
			if ( ! sendMsg(txMsg_))
			{
				INC_ERROR_COUNTER(canLogicErrorCount_);
				canStatus_ |= CAN_STATUS_TX_FAILED;
			}
		}
		else
		{
			MC_LOG_ERROR("Invalid MSG_PROT length %d", data.size());
			return;
		}
		break;
	}// END -- MSG_PROT handling

	default:
		MC_LOG_ERROR("Unimplemented EXT_MSG type: %d", data[0]);
		break;
	}
}

bool
Device::sendMsg(
	const HAL::CAN_Msg & msg)
{
	auto rc = HAL::CAN_Bus::RetCode::OK;

#if LOG_CAN_TRAFFIC
		MC_LOG_INFO("BUS <<< MCU %s", fmtCAN_DebugStr(msg.id,msg.ext,msg.data.size(),msg.data.data()));
#endif

	/**
	 * Disable interrupts because we don't want to initiate any SPI
	 * transactions from arriving CAN message while we're trying to send
	 * messages out. The SPI library has a tendency to get stuck in
	 * SPI.transfer() while waiting for the SPIF interrupt flag to get set.
	 */
	MC_ATOMIC_START
	rc = canBus_->sendAny(msg,false);// false -> don't wait for send
	MC_ATOMIC_END

	return rc == HAL::CAN_Bus::RetCode::OK;
}

}// namespace - MegaCAN
