#include "MegaCAN_ExtDevice.h"

#include <avr/wdt.h>
#include <EEPROM.h>

namespace MegaCAN
{

// a place to temporarily store modified pages of flash
uint8_t tempPage[MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE];

ExtDevice::ExtDevice(
		uint8_t cs,
		uint8_t myId,
		uint8_t intPin,
		CAN_Msg *buff,
		uint8_t buffSize,
		const TableDescriptor_t *tables,
		uint8_t numTables)
 : MegaCAN::Device(cs,myId,intPin,buff,buffSize)
 , tables_(tables)
 , numTables_(numTables)
 , currFlashTable_(numTables)
 , needsBurn_(false)
 , flashDataLost_(false)
 , onTableWrittenCallback_(nullptr)
 , onTableBurnedCallback_(nullptr)
{
	memset(dirtyFlashBits_,0,MEGA_CAN_EXT_NUM_DIRTY_FLASH_WORDS);// reset all dirty bits
}

bool
ExtDevice::readFromTable(
		const uint8_t &table,
		const uint16_t &offset,
		const uint8_t &len,
		const uint8_t *&resData)
{
	DEBUG(
		"readFromTable - table = %d; offset = %d; len = %d",
		table,
		offset,
		len);

	if (table > numTables_)
	{
		ERROR("%d > numTables_", table);
		return false;
	}

	const TableDescriptor_t &td = tables_[table];
	if (td.tableData == nullptr)
	{
		ERROR("null table %d", table);
		return false;
	}
	else if ((offset + len) > td.tableSize)
	{
		ERROR("outofbound - table %d; offset; %d; len; %d", table, offset, len);
		return false;
	}

	// see if we need to load in a new table of flash
	if (td.tableType == TableType_E::eFlash && currFlashTable_ != table)
	{
		if ( ! loadFlashTable(table))
		{
			return false;
		}
	}

	// return pointer to data within table
	// Note: when tables are loaded in from flash tableData will point to RAM
	resData = (uint8_t*)(td.tableData) + offset;
		
	return true;
}

bool
ExtDevice::writeToTable(
	const uint8_t &table,
	const uint16_t &offset,
	const uint8_t &len,
	const uint8_t *data)
{
	DEBUG(
		"writeToTable - table = %d; offset = %d; len = %d",
		table,
		offset,
		len);

	if (table > numTables_)
	{
		ERROR("%d > numTables_", table);
		return false;
	}

	const TableDescriptor_t &td = tables_[table];
	if (td.tableData == nullptr)
	{
		ERROR("null table %d", table);
		return false;
	}

	if ((offset + len) > td.tableSize)
	{
		ERROR("outofbound - table %d; offset; %d; len; %d", table, offset, len);
		return false;
	}

	// see if we need to load in a new table of flash
	if (td.tableType == TableType_E::eFlash && currFlashTable_ != table)
	{
		if ( ! loadFlashTable(table))
		{
			return false;
		}
	}

	// write data to temporary RAM location where flash was loaded
	needsBurn_ = (td.tableType == TableType_E::eFlash) && (len > 0);
	uint16_t flashOffset = offset;
	for (uint8_t i=0; i<len; i++)
	{
		dirtyFlashBits_[flashOffset/8] |= 1<<(flashOffset%8);// update dirty bits
		((uint8_t*)(td.tableData))[flashOffset] = data[i];
		flashOffset++;
	}

	// notify optional user callback
	if (onTableWrittenCallback_)
	{
		onTableWrittenCallback_(table,offset,len,data);
	}
		
	return true;
}

bool
ExtDevice::burnTable(
		const uint8_t &table)
{
	if (table != currFlashTable_)
	{
		ERROR("table to burn is %d but current is %d", table, currFlashTable_);
		return false;
	}
	else if (table >= numTables_)
	{
		ERROR("invalid table %d", table);
		return false;
	}
	else if ( ! needsBurn_)
	{
		// be nice and don't use unecessary write cycles on flash
		WARN("flash was never modified. ignoring burn request");
		return true;
	}
	
	const TableDescriptor_t &td = tables_[table];
	if (td.tableType != TableType_E::eFlash)
	{
		ERROR("requested burn to non-flash table %d", table);
		return false;
	}
	
	// write modified table contents from RAM to flash
	for (unsigned int i=0; i<td.tableSize; i++)
	{
		if (dirtyFlashBits_[i/8] & (1<<(i%8)))
		{
			// writing to EEPROM is slow (~3.4ms per byte)
			// keep watchdog happy if user code uses it
			wdt_reset();

			EEPROM.write(td.flashOffset + i, ((uint8_t*)(td.tableData))[i]);
		}
	}
	needsBurn_ = false;
	memset(dirtyFlashBits_,0,MEGA_CAN_EXT_NUM_DIRTY_FLASH_WORDS);// reset all dirty bits

	// notify optional user callback
	if (onTableBurnedCallback_)
	{
		onTableBurnedCallback_(table);
	}

	return true;
}

bool
ExtDevice::loadFlashTable(
	const uint8_t &table)
{
	if (needsBurn_)
	{
		// TODO could be nice and write to flash, but hoping TunerStudio
		// is smart enough to not put us in here. We'll see...
		WARN("unburned flash contents being lost");
		flashDataLost_ = true;
	}

	// load flash table contents into RAM
	const TableDescriptor_t &td = tables_[table];
	for (unsigned int i=0; i<td.tableSize; i++)
	{
		((uint8_t*)(td.tableData))[i] = EEPROM.read(td.flashOffset + i);
	}
	currFlashTable_ = table;
	needsBurn_ = false;
	memset(dirtyFlashBits_,0,MEGA_CAN_EXT_NUM_DIRTY_FLASH_WORDS);// reset all dirty bits
	DEBUG("loaded %dbytes from flash table %d", td.tableSize, table);
	
	return true;
}

}// namespace - MegaCAN
