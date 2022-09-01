#ifndef MEGA_CAN_BASE_H_
#define MEGA_CAN_BASE_H_

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdint.h>

#include "logging.h"
#include "MSG_defn.h"

#include <util/atomic.h>
#define MC_ATOMIC_START ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#define MC_ATOMIC_END }

#define SET_MEGA_CAN_SIG(USER_SIG) \
	static_assert(sizeof(USER_SIG) <= MAX_SIGNATURE_BYTES, \
		"Serial signature must be less than 60chars, including the null terminator"); \
	const char* MegaCAN_Base::__MegaCAN_SerialSignature = (USER_SIG);

#define SET_MEGA_CAN_REV(USER_REV) \
	static_assert(sizeof(USER_REV) == 20, \
		"Serial revision must be 20chars long, including the null terminator. You can pad the revision with spaces."); \
	const char* MegaCAN_Base::__MegaCAN_SerialRevision = (USER_REV);

#define CAN_STATUS_RX_OVERFLOW 0x1
#define CAN_STATUS_TX_FAILED   0x2

struct CAN_Msg
{
	// 11bit or 29bit identifier
	uint32_t id;
	// true if id is 29bits
	uint8_t  ext;
	// the length of data in teh rxBuff
	uint8_t  len;
	// the payload attachment
	uint8_t  rxBuf[8];
};

class CAN_MsgQueue
{
public:
	CAN_MsgQueue(
		uint8_t size)
	{
		buff_ = (CAN_Msg*)malloc(sizeof(CAN_Msg) * size);
		buff_size_ = size;
		buff_owned_ = true;
		clear();
	}

	CAN_MsgQueue(
		CAN_Msg *buff,
		uint8_t size)
	{
		buff_ = buff;
		buff_size_ = size;
		buff_owned_ = false;
		clear();
	}

	~CAN_MsgQueue()
	{
		if (buff_owned_ && buff_ != NULL)
		{
			free(buff_);
			buff_ = NULL;
		}
	}

	void
	clear()
	{
		front_ = 0;
		back_ = 0;
		size_ = 0;
	}

	bool
	isFull() const
	{
		MC_ATOMIC_START
		return next(back_) == front_;
		MC_ATOMIC_END
	}

	bool
	isEmpty() const
	{
		MC_ATOMIC_START
		return size_ == 0;
		MC_ATOMIC_END
	}

	void
	push()
	{
		MC_ATOMIC_START
		if (next(back_) != front_)// make sure it's not full
		{
			back_ = next(back_);
			size_++;
		}
		MC_ATOMIC_END
	}

	void
	pop()
	{
		MC_ATOMIC_START
		if (size_ != 0)// make sure it's not empty
		{
			front_ = next(front_);
			size_--;
		}
		MC_ATOMIC_END
	}

	unsigned int
	size()
	{
		MC_ATOMIC_START
		if (back_ >= front_)
		{
			return back_ - front_;
		}
		else
		{
			return back_ + buff_size_ - front_;
		}
		MC_ATOMIC_END
	}

	uint8_t
	capacity() const
	{
		return buff_size_;
	}

	CAN_Msg *
	getFrontPtr()
	{
		return &buff_[front_];
	}

	CAN_Msg *
	getBackPtr()
	{
		return &buff_[back_];
	}

private:
	uint8_t
	next(
			uint8_t idx) const
	{
		// idx is a local copy, so it's safe to increment
		idx++;
		if (idx >= buff_size_)
		{
			idx = 0;
		}
		return idx;
	}

private:
	// block of CAN messages to use for the queue
	CAN_Msg *buff_;
	// the number of allocated elements in buff_
	uint8_t buff_size_;
	// set true if this class owns the alloced memory in buff_
	bool buff_owned_;

	volatile uint8_t front_;
	volatile uint8_t back_;
	volatile uint8_t size_;

};

struct MegaCAN_Options
{
	/**
	 * if set true, then the interrupt handler will cann the handleStandard()
	 * method immediately, rather than buffering the messages and waiting
	 * for a handle() call. If this is set true, the handleStandard() method
	 * should be very quick or else CAN samples will be dropped.
	 */
	bool handleStandardMsgsImmediately;
};

class MegaCAN_Base
{

public:
	MegaCAN_Base(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			uint8_t buffSize = 40);

	MegaCAN_Base(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			CAN_Msg *buff,
			uint8_t buffSize);

	MegaCAN_Base() = delete;

	virtual
	~MegaCAN_Base();

	void
	init();

	void
	interrupt();

	void
	handle();

	/**
	 * Writes an 11bit CAN frame immediately. Useful for transmitting broadcast data.
	 * 
	 * @param[in] id
	 * The 11bit CAN indentifier
	 * 
	 * @param[in] len
	 * Number of data bytes to transmit in the CAN frame (max of 8 per CAN)
	 * 
	 * @param[in] buf
	 * Pointer to the data to send
	 * 
	 * @return
	 * True if successful, false otherwise.
	 */
	bool
	send11bitFrame(
		uint16_t id,
		uint8_t len,
		uint8_t *buf);

	uint8_t
	canId() const
	{
		return myID_;
	}

	uint8_t
	getCAN_Status()
	{
		return canStatus_;
	}

	uint8_t
	getLogicErrorCount()
	{
		return canLogicErrorCount_;
	}

	uint8_t
	getSW_RxOverflowCount()
	{
		return canSW_RxOverflowCount_;
	}

	uint8_t
	getHW_Rx0_OverflowCount()
	{
		return canHW_Rx0_OverflowCount_;
	}

	uint8_t
	getHW_Rx1_OverflowCount()
	{
		return canHW_Rx1_OverflowCount_;
	}

	void
	resetErrorCounters()
	{
		canLogicErrorCount_ = 0;
		canSW_RxOverflowCount_ = 0;
		canHW_Rx0_OverflowCount_ = 0;
		canHW_Rx1_OverflowCount_ = 0;
	}

protected:
	/**
	 * Overridable method that allow subclasses to provided custom
	 * options to the base class.
	 * 
	 * @param[in] opts
	 * The options that can be set
	 */
	virtual void
	getOptions(
		struct MegaCAN_Options *opts);

	/**
	 * Overridable method that allow subclasses to provided custom CAN
	 * filters that get loaded into the MCP2515 hardware filters.
	 * 
	 * @param[in] can
	 * The MCP2515 CAN interface library to apply the filters to
	 */
	virtual void
	applyCanFilters(
		MCP_CAN *can);

	/**
	 * Overridable method for subclass to implement. This method is called by
	 * the base class when informing Megasquirt/TunerStudio of the maximum
	 * table block read size.
	 *
	 * @return
	 * Maximum allowable table block read size in bytes
	 */
	virtual uint16_t
	tableBlockingFactor();

	/**
	 * Overridable method for subclass to implement. This method is called by
	 * the base class when informing Megasquirt/TunerStudio of the maximum
	 * flash table block read/write size.
	 *
	 * @return
	 * Maximum allowable flash table block read/write size in bytes
	 */
	virtual uint16_t
	writeBlockingFactor();

	/**
	 * Overridable method for subclass to implement. This method is called by
	 * the base class when a corresponding read request was made to this CAN
	 * device.
	 *
	 * @param[in] table
	 * The table index to read from
	 *
	 * @param[in] offset
	 * The byte offset to begin reading from (relative to the table's start)
	 *
	 * @param[in] len
	 * The number of bytes to read
	 *
	 * @param[out] resData
	 * A pointer to the corresponding memory to read from
	 *
	 * @return
	 * True if the read is valid, false if not.
	 */
	virtual bool
	readFromTable(
			const uint8_t &table,
			const uint16_t &offset,
			const uint8_t &len,
			const uint8_t *&resData);

	/**
	 * Overridable method for subclass to implement. This method is called by
	 * the base class when a corresponding write command was made to this CAN
	 * device.
	 *
	 * @param[in] table
	 * The table index to write to
	 *
	 * @param[in] offset
	 * The byte offset to begin writing to (relative to the table's start)
	 *
	 * @param[in] len
	 * The number of bytes to write
	 *
	 * @param[out] data
	 * A pointer to the data to write
	 *
	 * @return
	 * True if the write was successful, false if not.
	 */
	virtual bool
	writeToTable(
			const uint8_t &table,
			const uint16_t &offset,
			const uint8_t &len,
			const uint8_t *data);

	/**
	 * Overridable method for subclass to perform flash table burn. The base
	 * class will handle the MSG_BURNACK logic based on the return value of
	 * this method.
	 * 
	 * @param[in] table
	 * The requested table to burn the modified contents of flash to
	 * 
	 * @return
	 * True if the burn was successful, false if not. Used to send MSG_BURNACK
	 */
	virtual bool
	burnTable(
			const uint8_t &table);

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
			uint8_t *data);

	void
	simReqDrop(
		uint8_t numReqsToDrop);

private:
	void
	setupOptions();
	
	/**
	 * Called when an extended 29bit megasquirt frame is received.
	 * 
	 * @param[in] hdr
	 * The header of the CAN frame (reinterpreted 29bit identifier)
	 * 
	 * @param[in] length
	 * The number of data bytes in the CAN frame
	 * 
	 * @param[in] data
	 * A pointer to the data segment of the CAN frame
	 */
	void
	handleExtended(
			const MS_HDR_t *hdr,
			const uint8_t &length,
			uint8_t *data);

	/**
	 * Called when a megasquirt MSG_REQ frame is received.
	 * 
	 * @param[in] hdr
	 * The header of the CAN frame (reinterpreted 29bit identifier)
	 * 
	 * @param[in] reqData
	 * A pointer to the data segment of the request frame
	 */
	void
	handleRequest(
			const MS_HDR_t *hdr,
			uint8_t *reqData);

	/**
	 * Called within CAN ISR when a Megasquirt extended message is received.
	 * 
	 * @param[in] hdr
	 * The 29bit CAN identifier for the extended message
	 * 
	 * @param[in] length
	 * The length of the data section of the CAN message
	 * 
	 * @param[in/out] data
	 * Pointer to the CAN message data
	 */
	void
	handleExtendedMsg(
			const MS_HDR_t *hdr,
			const uint8_t &length,
			uint8_t *data);

	/**
	 * Writes a CAN frame
	 * 
	 * @param[in] id
	 * The 29bit or 11bit CAN indentifier
	 * 
	 * @param[in] ext
	 * 1 if frame if extended (29bit), or 0 if frame is standard (11bit)
	 * 
	 * @param[in] len
	 * Number of data bytes to transmit in the CAN frame (max of 8 per CAN)
	 * 
	 * @param[in] buf
	 * Pointer to the data to send
	 * 
	 * @return
	 * True if successful, false otherwise.
	 */
	bool
	sendMsgBuf(
		uint32_t id,
		uint8_t ext,
		uint8_t len,
		uint8_t *buf);

public:
	static const char* __MegaCAN_SerialSignature;
	static const char* __MegaCAN_SerialRevision;

private:
	MCP_CAN can_;
	uint8_t myID_;
	// MCP2515 interrupt pin
	// active low
	uint8_t intPin_;

	struct MegaCAN_Options opts_;

	// CAN RX Variables
	CAN_MsgQueue queue_;

	// Status word for the CAN interface.
	// see CAN_STATUS_* defines
	volatile uint8_t canStatus_;

	// allow error handler callbacks to modify private error counters
	friend void mega_can_rx0_ovr(MCP_CAN *can, void *varg);
	friend void mega_can_rx1_ovr(MCP_CAN *can, void *varg);

	// Total number of CAN errors detected (counters saturate)
	volatile uint8_t canLogicErrorCount_;
	volatile uint8_t canSW_RxOverflowCount_;
	volatile uint8_t canHW_Rx0_OverflowCount_;
	volatile uint8_t canHW_Rx1_OverflowCount_;

	// debug feature to drop the next N req messages (don't send RSP)
	uint8_t numSimReqDropsLeft_;

	// Buffer of CAN frame data used for building responses
	uint8_t txBuf_[8];

	// Header structure used for building CAN responses
	MS_HDR_t rspHdr_;

};

#endif
