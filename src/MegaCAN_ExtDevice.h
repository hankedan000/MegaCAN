#pragma once

#include "MegaCAN_ExtTypes.h"
#include "MegaCAN_Device.h"

namespace MegaCAN
{

// the largest flash table size allowed to be stored in flash
#define MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE 128
#define MEGA_CAN_EXT_NUM_DIRTY_FLASH_WORDS (MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE / 8)// 8bits per bytes

static_assert((MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE % 8) == 0,
		"Max flash table size must be a multiple of 8 bytes");

// a place to temporarily store modified pages of flash
extern uint8_t tempPage[MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE];

class ExtDevice : public MegaCAN::Device
{
public:
	using OnTableWrittenCallback = void (*)(
		uint8_t table,
		uint16_t offset,
		const HAL::CAN_DataBuffer & data);
	using OnTableBurnedCallback = void (*)(
		uint8_t table);
	
public:
	ExtDevice() = default;
	
	ExtDevice(
			const SharedPtr<HAL::CAN_Bus> & canBus,
			const uint8_t myMsqId,
			const TableDescriptor_t * tables,
			uint8_t numTables);

	bool
	needsBurn() const
	{
		return needsBurn_;
	}

	bool
	flashDataLost() const
	{
		return flashDataLost_;
	}

	uint8_t
	currFlashTable() const
	{
		return currFlashTable_;
	}

	void
	setOnTableWrittenCallback(
		OnTableWrittenCallback cb)
	{
		onTableWrittenCallback_ = cb;
	}

	void
	setOnTableBurnedCallback(
		OnTableBurnedCallback cb)
	{
		onTableBurnedCallback_ = cb;
	}

protected:
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
	bool
	readFromTable(
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		HAL::CAN_DataBuffer & dataOut) final;

	bool
	writeToTable(
		const uint8_t table,
		const uint16_t offset,
		const HAL::CAN_DataBuffer & data) final;

	// called by base class when we should burn the current flash table
	bool
	burnTable(
			const uint8_t table) final;
	
	uint16_t
	tableBlockingFactor() final
	{
		return 32;
	}
	
	uint16_t
	writeBlockingFactor() final
	{
		return 32;
	}

private:
	bool
	loadFlashTable(
		const uint8_t table);

private:
	/**
	 * An array of table descriptors used to read/write data to RAM
	 * or flash.
	 */
	const TableDescriptor_t *tables_;
	uint8_t numTables_;
	
	/**
	 * The current table of flash that's loaded into RAM
	 */
	uint8_t currFlashTable_;
	
	/**
	 * flag set true when a flash table has been loaded into RAM and
	 * modified, but not yet burned. This flag is cleared once the
	 * flash table has been burned via a MSG_BURN request.
	 */
	bool needsBurn_;

	/**
	 * A bitmask of which bytes in the active flash table have been
	 * modified in RAM, but not yet burned. This table is used to
	 * only burn the words that were modified in effort to reduce
	 * the total number of write cycles we put on the EEPROM.
	 */
	uint8_t dirtyFlashBits_[MEGA_CAN_EXT_NUM_DIRTY_FLASH_WORDS];

	/**
	 * set to true when a flash table was modified and then swapped out
	 * for another table prior to receiving a MSG_BURN request
	 */
	bool flashDataLost_;

	// optional user callbacks when a table has been written/burned
	OnTableWrittenCallback onTableWrittenCallback_;
	OnTableBurnedCallback onTableBurnedCallback_;

};

}// namespace - MegaCAN
