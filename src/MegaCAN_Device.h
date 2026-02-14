#pragma once

#include <string.h>
#include <stdint.h>

#include "MegaCAN/hal/CAN_Bus.h"
#include "MegaCAN/Logging.h"
#include "MegaCAN/Memory.h"
#include "MegaCAN/StaticVector.h"
#include "MSG_defn.h"

#define DECL_MEGA_CAN_REV(USER_REV) \
	static_assert(sizeof(USER_REV) <= MAX_REVISION_BYTES, \
		"Serial revision must be less than 60chars, including the null terminator"); \
	const uint8_t MegaCAN::Device::__MegaCAN_SerialRevisionLen = sizeof(USER_REV); \
	const char* MegaCAN::Device::__MegaCAN_SerialRevision = (USER_REV);

#define DECL_MEGA_CAN_SIG(USER_SIG) \
	static_assert(sizeof(USER_SIG) == MAX_SIGNATURE_BYTES, \
		"Serial signature must be 20chars long, including the null terminator. You can pad the signature with spaces."); \
	const char* MegaCAN::Device::__MegaCAN_SerialSignature = (USER_SIG);

#define CAN_STATUS_RX_OVERFLOW 0x1
#define CAN_STATUS_TX_FAILED   0x2

namespace MegaCAN
{

struct Options
{
	/**
	 * if set true, then the interrupt handler will call the handleStandard()
	 * method immediately, rather than buffering the messages and waiting
	 * for a handle() call. If this is set true, the handleStandard() method
	 * should be very quick or else CAN samples will be dropped.
	 */
	bool handleStandardMsgsImmediately;
};

class Device
{
public:
	Device() = delete;

	Device(
		const SharedPtr<HAL::CAN_Bus> & canBus,
		const uint8_t myMsqId);

	virtual ~Device() = default;

	void
	init();

	void
	interrupt();

	void
	handle();

	uint8_t
	msqId() const
	{
		return myMsqId_;
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
		struct Options *opts);

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
	 * @param[out] dataOut
	 * Buffer of data that was read
	 *
	 * @return
	 * True if the read is valid, false if not.
	 */
	virtual bool
	readFromTable(
			const uint8_t table,
			const uint16_t offset,
			const uint8_t len,
			HAL::CAN_DataBuffer & dataOut);

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
	 * @param[out] data
	 * Buffer of data to write
	 *
	 * @return
	 * True if the write was successful, false if not.
	 */
	virtual bool
	writeToTable(
			const uint8_t table,
			const uint16_t offset,
			const HAL::CAN_DataBuffer & data);

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
			const uint8_t table);

	/**
	 * Called when a standard 11bit megasquirt broadcast frame is received.
	 * 
	 * @param[in] id
	 * The 11bit CAN identifier
	 * 
	 * @param[in] data
	 * The data segment of the CAN frame
	 */
	virtual void
	handleStandard(
			const uint32_t id,
			const HAL::CAN_DataBuffer & data);

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
	 * @param[in] data
	 * The data segment of the CAN frame
	 */
	void
	handleExtended(
			const MS_HDR_t *hdr,
			const HAL::CAN_DataBuffer & data);

	/**
	 * Called when a megasquirt MSG_REQ frame is received.
	 * 
	 * @param[in] hdr
	 * The header of the CAN frame (reinterpreted 29bit identifier)
	 * 
	 * @param[in] reqData
	 * The data segment from the CAN_Msg
	 */
	void
	handleRequest(
			const MS_HDR_t * hdr,
			const HAL::CAN_DataBuffer & reqData);

	/**
	 * Called within CAN ISR when a Megasquirt extended message is received.
	 * 
	 * @param[in] hdr
	 * The 29bit CAN identifier for the extended message
	 * 
	 * @param[in] length
	 * The length of the data section of the CAN message
	 * 
	 * @param[in] reqData
	 * The data segment from the CAN_Msg
	 */
	void
	handleExtendedMsg(
			const MS_HDR_t *hdr,
			const HAL::CAN_DataBuffer & data);

	/**
	 * Writes a CAN msg
	 * 
	 * @param[in] msg
	 * The message to send
	 * 
	 * @return
	 * True if successful, false otherwise.
	 */
	bool
	sendMsg(
		const HAL::CAN_Msg & msg);

public:
	static const char* __MegaCAN_SerialSignature;
	static const uint8_t __MegaCAN_SerialRevisionLen;
	static const char* __MegaCAN_SerialRevision;

private:
	SharedPtr<HAL::CAN_Bus> canBus_ = nullptr;
	uint8_t myMsqId_ = 0u;

	struct Options opts_;

	// CAN RX Variables
	HAL::CAN_MsgQueue queue_;

	// Status word for the CAN interface.
	// see CAN_STATUS_* defines
	volatile uint8_t canStatus_ = 0u;

	// Total number of CAN errors detected (counters saturate)
	volatile uint8_t canLogicErrorCount_ = 0u;
	volatile uint8_t canSW_RxOverflowCount_ = 0u;
	volatile uint8_t canHW_Rx0_OverflowCount_ = 0u;
	volatile uint8_t canHW_Rx1_OverflowCount_ = 0u;

	// debug feature to drop the next N req messages (don't send RSP)
	uint8_t numSimReqDropsLeft_ = 0u;

	// CAN_Msg used for building responses
	HAL::CAN_Msg txMsg_;

	// Header structure used for building CAN responses
	MS_HDR_t rspHdr_;

};

}// namespace - MegaCAN
