#ifndef MEGACAN_EXT_DEVICE_H_
#define MEGACAN_EXT_DEVICE_H_

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
	ExtDevice(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			CAN_Msg *buff,
			uint8_t buffSize,
			const TableDescriptor_t *tables,
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
	virtual bool
	readFromTable(
			const uint8_t &table,
			const uint16_t &offset,
			const uint8_t &len,
			const uint8_t *&resData) override;

	virtual bool
	writeToTable(
		const uint8_t &table,
		const uint16_t &offset,
		const uint8_t &len,
		const uint8_t *data) override;

	// called by base class when we should burn the current flash table
	virtual bool
	burnTable(
			const uint8_t &table) override;
	
	virtual uint16_t
	tableBlockingFactor() override
	{
		return 32;
	}
	
	virtual uint16_t
	writeBlockingFactor() override
	{
		return 32;
	}

private:
	bool
	loadFlashTable(
		const uint8_t &table);

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

};

}// namespace - MegaCAN

#endif
