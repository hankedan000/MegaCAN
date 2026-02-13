#include "MegaCAN_ExtDevice.h"

#include <EEPROM.h>
#include "MegaCAN/Watchdog.h"

namespace MegaCAN
{

// a place to temporarily store modified pages of flash
uint8_t tempPage[MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE];

ExtDevice::ExtDevice(
		const SharedPtr<HAL::CAN_Bus> & canBus,
		const uint8_t myMsqId,
		const TableDescriptor_t *tables,
		uint8_t numTables)
 : MegaCAN::Device(canBus, myMsqId)
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
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		HAL::CAN_DataBuffer & dataOut)
{
	MC_LOG_DEBUG(
		"readFromTable - table = %d; offset = %d; len = %d",
		table,
		offset,
		len);

	if (table > numTables_)
	{
		MC_LOG_ERROR("%d > numTables_", table);
		return false;
	}

	const TableDescriptor_t &td = tables_[table];
	if (td.tableData == nullptr)
	{
		MC_LOG_ERROR("null table %d", table);
		return false;
	}
	else if ((offset + len) > td.tableSize)
	{
		MC_LOG_ERROR("outofbound - table %d; offset; %d; len; %d", table, offset, len);
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

	// copy the tabel data into the response data buffer
	// Note: when tables are loaded in from flash tableData will point to RAM
	dataOut.resize_nofill(len);
	auto readFromPtr = static_cast<uint8_t *>(td.tableData) + offset;
	for (uint8_t i=0u; i<len; i++, readFromPtr++)
	{
		dataOut[i] = *readFromPtr;
	}
	return true;
}

bool
ExtDevice::writeToTable(
	const uint8_t table,
	const uint16_t offset,
	const HAL::CAN_DataBuffer & data)
{
	MC_LOG_DEBUG(
		"writeToTable - table = %d; offset = %d; len = %d",
		table,
		offset,
		len);

	if (table > numTables_)
	{
		MC_LOG_ERROR("%d > numTables_", table);
		return false;
	}

	const TableDescriptor_t &td = tables_[table];
	if (td.tableData == nullptr)
	{
		MC_LOG_ERROR("null table %d", table);
		return false;
	}

	if ((offset + data.size()) > td.tableSize)
	{
		MC_LOG_ERROR("outofbound - table %d; offset; %d; len; %d", table, offset, data.size());
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
	needsBurn_ = (td.tableType == TableType_E::eFlash) && (data.size() > 0);
	uint16_t flashOffset = offset;
	for (uint8_t i=0; i<data.size(); i++)
	{
		dirtyFlashBits_[flashOffset/8] |= 1<<(flashOffset%8);// update dirty bits
		((uint8_t*)(td.tableData))[flashOffset] = data[i];
		flashOffset++;
	}

	// notify optional user callback
	if (onTableWrittenCallback_)
	{
		onTableWrittenCallback_(table, offset, data);
	}
		
	return true;
}

bool
ExtDevice::burnTable(
		const uint8_t table)
{
	if (table != currFlashTable_)
	{
		MC_LOG_ERROR("table to burn is %d but current is %d", table, currFlashTable_);
		return false;
	}
	else if (table >= numTables_)
	{
		MC_LOG_ERROR("invalid table %d", table);
		return false;
	}
	else if ( ! needsBurn_)
	{
		// be nice and don't use unecessary write cycles on flash
		MC_LOG_WARN("flash was never modified. ignoring burn request");
		return true;
	}
	
	const TableDescriptor_t &td = tables_[table];
	if (td.tableType != TableType_E::eFlash)
	{
		MC_LOG_ERROR("requested burn to non-flash table %d", table);
		return false;
	}
	
	// write modified table contents from RAM to flash
	for (unsigned int i=0; i<td.tableSize; i++)
	{
		if (dirtyFlashBits_[i/8] & (1<<(i%8)))
		{
			// writing to EEPROM is slow (~3.4ms per byte)
			// keep watchdog happy if user code uses it
			resetWatchdog();

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
	const uint8_t table)
{
	if (needsBurn_)
	{
		// TODO could be nice and write to flash, but hoping TunerStudio
		// is smart enough to not put us in here. We'll see...
		MC_LOG_WARN("unburned flash contents being lost");
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
	MC_LOG_DEBUG("loaded %dbytes from flash table %d", td.tableSize, table);
	
	return true;
}

}// namespace - MegaCAN
