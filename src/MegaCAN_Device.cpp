#include "MegaCAN_Device.h"

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

void mega_can_rx0_ovr(MCP_CAN *can, void *varg)
{
	MegaCAN::Device *mcd = (MegaCAN::Device*)varg;
	INC_ERROR_COUNTER(mcd->canHW_Rx0_OverflowCount_);
}

void mega_can_rx1_ovr(MCP_CAN *can, void *varg)
{
	MegaCAN::Device *mcd = (MegaCAN::Device*)varg;
	INC_ERROR_COUNTER(mcd->canHW_Rx1_OverflowCount_);
}

void mega_can_tx_bo(MCP_CAN *can, void *varg)
{
	MC_LOG_ERROR("TXBO");
}

void mega_can_tx_ep(MCP_CAN *can, void *varg, uint8_t tec)
{
	MC_LOG_ERROR("TXEP (%d)",tec);
}

void mega_can_rx_ep(MCP_CAN *can, void *varg, uint8_t rec)
{
	MC_LOG_ERROR("RXEP (%d)",rec);
}

void mega_can_tx_war(MCP_CAN *can, void *varg, uint8_t tec)
{
	MC_LOG_ERROR("TXWAR (%d)",tec);
}

void mega_can_rx_war(MCP_CAN *can, void *varg, uint8_t rec)
{
	MC_LOG_ERROR("RXWAR (%d)",rec);
}

static struct MCP_ErrorHandlers megaCAN_ErrHandlers{
	.rx0_ovr = mega_can_rx0_ovr,
	.rx1_ovr = mega_can_rx1_ovr,
	.tx_bo   = mega_can_tx_bo,
	.tx_ep   = mega_can_tx_ep,
	.rx_ep   = mega_can_rx_ep,
	.tx_war  = mega_can_tx_war,
	.rx_war  = mega_can_rx_war,
	.e_warn  = NULL
};

Device::Device(
		uint8_t cs,
		uint8_t myId,
		uint8_t intPin,
		CAN_Msg *buff,
		uint8_t buffSize)
	: can_(cs)
	, myID_(myId)
	, intPin_(intPin)
	, queue_(buff,buffSize)
	, canStatus_(0x0)
	, numSimReqDropsLeft_(0)
{
	resetErrorCounters();
	setupOptions();
}

Device::~Device()
{
}

void
Device::init()
{
	bool okay = true;

	// Initialize MCP2515 with a baudrate of 500kb/s
	if(okay && can_.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
	{
		MC_LOG_ERROR("MCP2515::begin() failed!");
		okay = false;
	}

	/**
	 * setup hardware-based CAN filters
	 * 
	 * The default will filter by the Megasquirt CAN device ID, but the
	 * subclass override the method to provide more custom filters.
	 */
	applyCanFilters(&can_);

	// set the mode to "NORMAL" (only mode we can RX & TX in)
	if (okay && can_.setMode(MCP_NORMAL) != CAN_OK)
	{
		MC_LOG_ERROR("MCP2515::setMode() failed!");
		okay = false;
	}

	if (okay)
	{
		MC_LOG_INFO("MegaCAN initialized!");
		MC_LOG_INFO("%d Rx CAN_Msg buffer (%dbytes)",
				queue_.capacity(),
				queue_.capacity() * sizeof(CAN_Msg));
	}
	else
	{
		MC_LOG_ERROR("MegaCAN initialization failed!");
	}
}

void
Device::interrupt()
{
	CAN_Msg *msg = queue_.getBackPtr();

	// TODO just make readMsgBuf() take a CAN_Msg instead of sneaking around it here
	uint8_t msgLen = 0;
	while (can_.readMsgBuf(&msg->id,&msg->ext,&msgLen,msg->data.data()) == CAN_OK)
	{
		// see if we should handle the broadcast messages immediately
		if (opts_.handleStandardMsgsImmediately && msg->ext == 0)
		{
			handleStandard(msg->id,msgLen,msg->data.data());
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

	can_.serviceErrors((void*)this,&megaCAN_ErrHandlers);
}

void
Device::handle()
{
	const CAN_Msg *msg = nullptr;
	while (!queue_.isEmpty())
	{
		msg = queue_.getFrontPtr();
#if LOG_CAN_TRAFFIC
		MC_LOG_INFO("BUS >>> MCU %s", fmtCAN_DebugStr(msg->id,msg->ext,msg->len,msg->data.data()));
#endif

		if (msg->ext)
		{
			const MS_HDR_t* hdr = reinterpret_cast<const MS_HDR_t*>(&msg->id);

			if(hdr->toId == myID_)
			{
				handleExtended(hdr,msg->data.size(),msg->data.data());
			}
			else
			{
				MC_LOG_WARN("msg not meant for me!!!");
			}
		}
		else
		{
			handleStandard(msg->id,msg->data.size(),msg->data.data());
		}

		queue_.pop();
	}
}

bool
Device::send11bitFrame(
	uint16_t id,
	uint8_t len,
	uint8_t *buf)
{
	return sendMsgBuf(id,0,len,buf);
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

void
Device::applyCanFilters(
	MCP_CAN *can)
{
	// setup the CAN mask/filters
	uint32_t mask = 0x0;
	uint32_t filt = 0x0;
	MS_HDR_t *maskHdr = reinterpret_cast<MS_HDR_t*>(&mask);
	MS_HDR_t *filtHdr = reinterpret_cast<MS_HDR_t*>(&filt);
	maskHdr->toId = 0xf;// only check the 4bit toId in the megasquirt header
	filtHdr->toId = myID_ & 0xf;// make sure the message is for me!
	can->init_Mask(0,1,mask);
	can->init_Mask(1,1,mask);
	can->init_Filt(0,1,filt);
	can->init_Filt(1,1,filt);
	can->init_Filt(2,1,filt);
	can->init_Filt(3,1,filt);
	can->init_Filt(4,1,filt);
	can->init_Filt(5,1,filt);
}

bool
Device::readFromTable(
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		const uint8_t *&resData)
{
	MC_LOG_WARN("subclass should override readFromTable()");
	return false;
}

bool
Device::writeToTable(
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		const uint8_t *data)
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
		const uint8_t length,
		uint8_t* data)
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
		const uint8_t length,
		uint8_t *data)
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
		writeToTable(table,hdr->offset,length,data);
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
		txBuf_[0] = MSG_BURNACK;
		txBuf_[1] = (burnOkay ? 1 : 0);

		if ( ! sendMsgBuf(rspHdr_.marshal(),true,2,txBuf_.data()))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
		break;
	}
	case MSG_XTND:
		handleExtendedMsg(hdr,length,data);
		break;
	default:
		MC_LOG_ERROR("Unimplemented MSG type: %d", hdr->type);
		break;
	}// switch
}

void
Device::handleRequest(
		const MS_HDR_t *hdr,
		uint8_t *reqData)
{
	bool okay = true;
	MSG_REQ_t* req = reinterpret_cast<MSG_REQ_t*>(reqData);
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

	const uint8_t *resData = NULL;
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
			resData = __MegaCAN_SerialSignature + hdr->offset;
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
				txBuf_[i] = __MegaCAN_SerialRevision[offset];
			}
			else
			{
				txBuf_[i] = '\0';
			}
		}
		resData = txBuf_.data();
	}
	else
	{
		okay = okay && readFromTable(
				reqTable,
				hdr->offset,
				req->rspLength,
				resData);
	}

	if (resData == NULL)
	{
		MC_LOG_ERROR("handleRequest - resData is NULL");
		okay = false;
	}

	rspHdr_.toId = hdr->fromId;
	rspHdr_.fromId = hdr->toId;
	rspHdr_.type = MSG_RSP;
	setTable(&rspHdr_,req->rspTable);
	rspHdr_.offset = rspOffset;

	if (okay)
	{
		// send MSG_RSP packet
		if ( ! sendMsgBuf(
				rspHdr_.marshal(),
				true,
				req->rspLength,
				const_cast<uint8_t*>(resData)))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
	}
	else
	{
		// handle response, but send back zeros
		txBuf_.resize(req->rspLength, 0u);
		if ( ! sendMsgBuf(rspHdr_.marshal(),true,txBuf_.size(),txBuf_.data()))
		{
			INC_ERROR_COUNTER(canLogicErrorCount_);
			canStatus_ |= CAN_STATUS_TX_FAILED;
		}
	}
}

void
Device::handleExtendedMsg(
		const MS_HDR_t *hdr,
		const uint8_t length,
		uint8_t *data)
{
	uint8_t rspLength;

	if (length < 1)
	{
		MC_LOG_ERROR("Invalid EXT MSG length %d", length);
		return;
	}

	switch(data[0])
	{

	case MSG_PROT:
	{
		if (length == 4)
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

			txBuf_[0] = 2;// serial version
			if (rspLength == 1)
			{
				// nothing else to set besides serial version
			}
			else if(rspLength == 5)
			{
				txBuf_[1] = tableBlockingFactor() >> 8;  // high byte
				txBuf_[2] = tableBlockingFactor() & 0xff;// low byte
				txBuf_[3] = writeBlockingFactor() >> 8;  // high byte
				txBuf_[4] = writeBlockingFactor() & 0xff;// low byte
			}
			else
			{
				MC_LOG_ERROR("Invalid MSG_PROT rspLength %d", rspLength);
				return;
			}

			// send protocol response
			if ( ! sendMsgBuf(rspHdr_.marshal(),true,rspLength,txBuf_.data()))
			{
				INC_ERROR_COUNTER(canLogicErrorCount_);
				canStatus_ |= CAN_STATUS_TX_FAILED;
			}
		}
		else
		{
			MC_LOG_ERROR("Invalid MSG_PROT length %d", length);
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
Device::sendMsgBuf(
	uint32_t id,
	uint8_t ext,
	uint8_t len,
	uint8_t *buf)
{
	uint8_t res = CAN_OK;

#if LOG_CAN_TRAFFIC
		MC_LOG_INFO("BUS <<< MCU %s", fmtCAN_DebugStr(id,ext,len,buf));
#endif

	/**
	 * Disable interrupts because we don't want to initiate any SPI
	 * transactions from arriving CAN message while we're trying to send
	 * messages out. The SPI library has a tendency to get stuck in
	 * SPI.transfer() while waiting for the SPIF interrupt flag to get set.
	 */
	MC_ATOMIC_START
	res = can_.sendMsgBuf(id,ext,len,buf,false);// false -> don't wait for send
	MC_ATOMIC_END

	return res == CAN_OK;
}

}// namespace - MegaCAN
