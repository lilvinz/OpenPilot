/**
 ******************************************************************************
 *
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FLASHFS_OBJLIST Object list based flash filesystem (low ram)
 * @{
 *
 * @file       pios_flashfs_objlist.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      A file system for storing UAVObject in flash chip
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#include "openpilot.h"
#include "uavobjectmanager.h"

#if defined(PIOS_INCLUDE_FLASH_COMPACT_SETTINGS)

// Private functions
static int32_t PIOS_FLASHFS_Compact_ClearObjectTableHeader();
static int32_t PIOS_FLASHFS_Compact_GetObjAddress(uint32_t objId, uint16_t instId);
static int32_t PIOS_FLASHFS_Compact_GetNewAddress(uint32_t objId, uint16_t instId);

// Private variables
static int32_t numObjects = -1;

// Private structures
// Header for objects in the file system table
struct objectHeader {
	uint32_t objMagic;
	uint32_t objId;
	uint32_t instId;
	uint32_t address;
} __attribute__((packed));;

struct fileHeader {
	uint32_t id;
	uint16_t instId;
	uint16_t size;
} __attribute__((packed));


#define MAX_BADMAGIC       1000

static const struct flashfs_compact_cfg * cfg;
/**
 * @brief Initialize the flash object setting FS
 * @return 0 if success, -1 if failure
 */
int32_t PIOS_FLASHFS_Compact_Init(const struct flashfs_compact_cfg * new_cfg)
{
	cfg = new_cfg;
	
	if (PIOS_Flash_Internal_StartTransaction() != 0)
		return -1;

	// Check for valid object table or create one
	uint32_t object_table_magic;
	uint32_t magic_fail_count = 0;
	bool magic_good = false;

	while (!magic_good) {
		if (PIOS_Flash_Internal_ReadData(cfg->addr_obj_table_magic, (uint8_t *)&object_table_magic, sizeof(object_table_magic)) != 0)
			return -1;
		if (object_table_magic != cfg->table_magic) {
			if (magic_fail_count++ > MAX_BADMAGIC) {

				//do the same as format, but dont use format as it is wrapped in transaction as well
				for (uint32_t addr = cfg->addr_chip_begin; addr < cfg->addr_chip_begin + cfg->chip_size; addr += cfg->sector_size)
					if (PIOS_Flash_Internal_EraseSector(addr) != 0) {
						PIOS_Flash_Internal_EndTransaction();
						return -1;
					}
				if (PIOS_FLASHFS_Compact_ClearObjectTableHeader() != 0) {
					PIOS_Flash_Internal_EndTransaction();
					return -1;
				}

#if defined(PIOS_LED_HEARTBEAT)
				PIOS_LED_Toggle(PIOS_LED_HEARTBEAT);
#endif	/* PIOS_LED_HEARTBEAT */
				magic_fail_count = 0;
				magic_good = true;
			} else {
				PIOS_DELAY_WaituS(1000);
			}
		}
		else {
			magic_good = true;
		}

	}

	int32_t addr = cfg->addr_obj_table_start;
	struct objectHeader header;
	numObjects = 0;

	// Loop through header area while objects detect to count how many saved
	while (addr < cfg->addr_obj_table_end) {
		// Read the instance data
		if (PIOS_Flash_Internal_ReadData(addr, (uint8_t *)&header, sizeof(header)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
			return -1;
		}

		// Counting number of valid headers
		if (header.objMagic != cfg->obj_magic)
			break;

		numObjects++;
		addr += sizeof(header);
	}

	if (PIOS_Flash_Internal_EndTransaction() != 0) {
		return -1;
	}

	return 0;
}

/**
 * @brief Erase the whole flash chip and create the file system
 * @return 0 if successful, -1 if not
 */
int32_t PIOS_FLASHFS_Compact_Format()
{
	if (PIOS_Flash_Internal_StartTransaction() != 0)
		return -1;

	for (uint32_t addr = cfg->addr_chip_begin; addr < cfg->addr_chip_begin + cfg->chip_size; addr += cfg->sector_size)
		if (PIOS_Flash_Internal_EraseSector(addr) != 0) {
			PIOS_Flash_Internal_EndTransaction();
			return -1;
		}

	if (PIOS_FLASHFS_Compact_ClearObjectTableHeader() != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -1;
	}

	if (PIOS_Flash_Internal_EndTransaction() != 0) {
		return -1;
	}

	return 0;
}

/**
 * @brief Erase the headers for all objects in the flash chip
 * @return 0 if successful, -1 if not
 */
static int32_t PIOS_FLASHFS_Compact_ClearObjectTableHeader()
{
	for (uint32_t addr = cfg->addr_obj_table_magic; addr < cfg->addr_obj_table_end; addr += cfg->sector_size)
		if (PIOS_Flash_Internal_EraseSector(addr) != 0)
			return -1;

	if (PIOS_Flash_Internal_WriteData(cfg->addr_obj_table_magic, (uint8_t *)&cfg->table_magic, sizeof(cfg->table_magic)) != 0)
		return -1;

	uint32_t object_table_magic;
	PIOS_Flash_Internal_ReadData(cfg->addr_obj_table_magic, (uint8_t *)&object_table_magic, sizeof(object_table_magic));
	if (object_table_magic != cfg->table_magic)
		return -1;

	return 0;
}

/**
 * @brief Get the address of an object
 * @param obj UAVObjHandle for that object
 * @parma instId Instance id for that object
 * @return address if successful, -1 if not found
 */
static int32_t PIOS_FLASHFS_Compact_GetObjAddress(uint32_t objId, uint16_t instId)
{
	int32_t addr = cfg->addr_obj_table_start;
	struct objectHeader header;

	// Loop through header area while objects detect to count how many saved
	while (addr < cfg->addr_obj_table_end) {
		// Read the instance data
		if (PIOS_Flash_Internal_ReadData(addr, (uint8_t *) &header, sizeof(header)) != 0)
			return -1;
		if (header.objMagic != cfg->obj_magic)
			break; // stop searching once hit first non-object header
		else if (header.objId == objId && header.instId == instId)
			break;
		addr += sizeof(header);
	}

	if (header.objId == objId && header.instId == instId)
		return header.address;

	return -1;
}

static uint32_t PIOS_FLASHFS_Compact_GetFileSize(UAVObjHandle objId)
{
	uint32_t objsize = 0;
	objsize += sizeof(struct fileHeader);
	if (objsize & 1) ++objsize;
	objsize += UAVObjGetNumBytes(objId);
	if (objsize & 1) ++objsize;
	objsize += sizeof(uint8_t);					//attention this has to be changed if the crc size is increased!
	if (objsize & 1) ++objsize;

	return objsize;
}

/**
 * @brief Returns an address for a new object and creates entry into object table
 * @param[in] obj Object handle for object to be saved
 * @param[in] instId The instance id of object to be saved
 * @return 0 if success or error code
 * @retval -1 Object not found
 * @retval -2 No room in object table
 * @retval -3 Unable to write entry into object table
 * @retval -4 FS not initialized
 * @retval -5
 */
int32_t PIOS_FLASHFS_Compact_GetNewAddress(uint32_t objId, uint16_t instId)
{
	struct objectHeader new_header;

	if (numObjects < 0)
		return -4;

	// Don't worry about max size of flash chip here, other code will catch that
	new_header.objMagic = cfg->obj_magic;
	new_header.objId = objId;
	new_header.instId = instId;
	new_header.address = cfg->addr_files;

	// Loop through header area while objects detect to find next free file address
	{
		struct objectHeader search_header;
		int32_t addr = cfg->addr_obj_table_start;
		while (addr < cfg->addr_obj_table_end) {

			// Read the instance data
			if (PIOS_Flash_Internal_ReadData(addr, (uint8_t *) &search_header, sizeof(search_header)) != 0)
				return -1;

			if (search_header.objMagic != cfg->obj_magic)
				break; // stop searching once hit first non-object header

			struct fileHeader search_file_header;
			if (PIOS_Flash_Internal_ReadData(search_header.address, (uint8_t *) &search_file_header, sizeof(search_file_header)) != 0)
				return -1;

			new_header.address += search_file_header.size;

			addr += sizeof(search_header);
		}
	}

	//now ensure that the newly created file fits into the currently active page
	{
		uint32_t available_in_current_page = cfg->sector_size - new_header.address % cfg->sector_size;
		if (PIOS_FLASHFS_Compact_GetFileSize(UAVObjGetByID(objId)) > available_in_current_page) {
			new_header.address = new_header.address - (new_header.address % cfg->sector_size) + cfg->sector_size;
		}
	}

	int32_t addr = cfg->addr_obj_table_start + sizeof(new_header) * numObjects;

	// No room for this header in object table
	if ((addr + sizeof(new_header)) > cfg->addr_obj_table_end)
		return -2;

	// no room for this object in file space
	if (new_header.address + PIOS_FLASHFS_Compact_GetFileSize(UAVObjGetByID(objId)) > cfg->addr_chip_begin + cfg->chip_size)
		return -2;

	//write out the header
	if (PIOS_Flash_Internal_WriteData(addr, (uint8_t *) &new_header, sizeof(new_header)) != 0)
		return -3;

	// This numObejcts value must stay consistent or there will be a break in the table
	// and later the table will have bad values in it
	numObjects++;
	return new_header.address;
}


/**
 * @brief Saves one object instance per sector
 * @param[in] obj UAVObjHandle the object to save
 * @param[in] instId The instance of the object to save
 * @return 0 if success or -1 if failure
 * @note This uses one sector on the flash chip per object so that no buffering in ram
 * must be done when erasing the sector before a save
 */
int32_t PIOS_FLASHFS_Compact_ObjSave(UAVObjHandle obj, uint16_t instId, uint8_t * data)
{
	uint32_t objId = UAVObjGetID(obj);
	uint8_t crc = 0;

	if (PIOS_Flash_Internal_StartTransaction() != 0)
		return -1;

	int32_t addr = PIOS_FLASHFS_Compact_GetObjAddress(objId, instId);

	// Object currently not saved
	if (addr < 0)
		addr = PIOS_FLASHFS_Compact_GetNewAddress(objId, instId);

	// Could not allocate space
	if (addr < 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -1;
	}

	struct fileHeader header = {
		.id = objId,
		.instId = instId,
		.size = PIOS_FLASHFS_Compact_GetFileSize(obj),
	};

	// Update CRC
	crc = PIOS_CRC_updateCRC(0, (uint8_t *) &header, sizeof(header));
	crc = PIOS_CRC_updateCRC(crc, (uint8_t *) data, UAVObjGetNumBytes(obj));

	//now this is ugly but necessary

	//erase scratchpad
	if (PIOS_Flash_Internal_EraseSector(cfg->addr_scratchpad) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -2;
	}

	//copy current sector into scratchpad
	uint32_t current_sector = addr - (addr % cfg->sector_size);
	for (uint32_t iByte = 0; iByte < cfg->sector_size; iByte += 2)
	{
		uint16_t temp;
		if (PIOS_Flash_Internal_ReadData(current_sector + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
		if (PIOS_Flash_Internal_WriteData(cfg->addr_scratchpad + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
	}

	//erase current sector
	if (PIOS_Flash_Internal_EraseSector(current_sector) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -2;
	}

	//now transfer bytes before current change back from scratchpad to current_sector
	for (uint32_t iByte = 0; iByte < addr - current_sector; iByte += 2)
	{
		uint16_t temp;
		if (PIOS_Flash_Internal_ReadData(cfg->addr_scratchpad + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
		if (PIOS_Flash_Internal_WriteData(current_sector + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
	}

	//next write the new data to the current sector
	struct pios_flash_chunk chunks[3] = {
		{
			.addr = (uint8_t *) &header,
			.len = sizeof(header),
		},
		{
			.addr = (uint8_t *) data,
			.len = UAVObjGetNumBytes(obj),
		},
		{
			.addr = (uint8_t *) &crc,
			.len = sizeof(crc),
		}
	};

	//write chunks will round len up to an even value for each chunk and fill with zeros
	if (PIOS_Flash_Internal_WriteChunks(addr, chunks, NELEMENTS(chunks)) != 0) {
		PIOS_Flash_Internal_EndTransaction();
	   return -1;
	}
	
	//and finally transfer the remaining data from scratchpad back to current_sector
	for (uint32_t iByte = (addr % cfg->sector_size) + header.size; iByte < cfg->sector_size; iByte += 2)
	{
		uint16_t temp;
		if (PIOS_Flash_Internal_ReadData(cfg->addr_scratchpad + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
		if (PIOS_Flash_Internal_WriteData(current_sector + iByte, (uint8_t *)&temp, sizeof(temp)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
		   return -1;
		}
	}

	if (PIOS_Flash_Internal_EndTransaction() != 0) {
		return -1;
	}

	return 0;
}

/**
 * @brief Load one object instance per sector
 * @param[in] obj UAVObjHandle the object to save
 * @param[in] instId The instance of the object to save
 * @return 0 if success or error code
 * @retval -1 if object not in file table
 * @retval -2 if unable to retrieve object header
 * @retval -3 if loaded data instId or objId don't match
 * @retval -4 if unable to retrieve instance data
 * @retval -5 if unable to read CRC
 * @retval -6 if CRC doesn't match
 * @note This uses one sector on the flash chip per object so that no buffering in ram
 * must be done when erasing the sector before a save
 */
int32_t PIOS_FLASHFS_Compact_ObjLoad(UAVObjHandle obj, uint16_t instId, uint8_t * data)
{
	uint32_t objId = UAVObjGetID(obj);
	uint16_t objSize = UAVObjGetNumBytes(obj);
	uint8_t crc = 0;
	uint8_t crcFlash = 0;

	if (PIOS_Flash_Internal_StartTransaction() != 0)
		return -1;

	int32_t addr = PIOS_FLASHFS_Compact_GetObjAddress(objId, instId);

	// Object currently not saved
	if (addr < 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -1;
	}

	struct fileHeader header;

	// Load header
	// This information IS redundant with the object table id.  Oh well.  Better safe than sorry.
	if (PIOS_Flash_Internal_ReadData(addr, (uint8_t *) &header, sizeof(header)) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -2;
	}
	
	// Update CRC
	crc = PIOS_CRC_updateCRC(0, (uint8_t *) &header, sizeof(header));

	if((header.id != objId) || (header.instId != instId)) {
		PIOS_Flash_Internal_EndTransaction();
		return -3;
	}
	
	// To avoid having to allocate the RAM for a copy of the object, we read by chunks
	// and compute the CRC
	uint32_t temp_addr = addr + sizeof(header);
	if (temp_addr & 1)
		++temp_addr;

	for (uint32_t i = 0; i < objSize; ++i) {
		uint8_t byte;
		if (PIOS_Flash_Internal_ReadData(temp_addr + i, &byte, 1) != 0) {
			PIOS_Flash_Internal_EndTransaction();
			return -4;
		}
		crc = PIOS_CRC_updateCRC(crc, &byte, 1);
	}

	temp_addr += objSize;
	if (temp_addr & 1)
		++temp_addr;

	// Read CRC (written so will work when CRC changes to uint16)
	if (PIOS_Flash_Internal_ReadData(temp_addr, (uint8_t *) &crcFlash, sizeof(crcFlash)) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -5;
	}

	if (crc != crcFlash) {
		PIOS_Flash_Internal_EndTransaction();
		return -6;
	}

	// Read the instance data
	temp_addr = addr + sizeof(header);
	if (temp_addr & 1)
		++temp_addr;
	if (PIOS_Flash_Internal_ReadData(temp_addr, data, objSize) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -4;
	}

	if (PIOS_Flash_Internal_EndTransaction() != 0)
		return -1;

	return 0;
}

/**
 * @brief Delete object from flash
 * @param[in] obj UAVObjHandle the object to save
 * @param[in] instId The instance of the object to save
 * @return 0 if success or error code
 * @retval -1 if object not in file table
 * @retval -2 Erase failed
 * @note To avoid buffering the file table (1k ram!) the entry in the file table
 * remains but destination sector is erased.  This will make the load fail as the
 * file header won't match the object.  At next save it goes back there.
 */
int32_t PIOS_FLASHFS_Compact_ObjDelete(UAVObjHandle obj, uint16_t instId)
{
	uint32_t objId = UAVObjGetID(obj);

	if (PIOS_Flash_Internal_StartTransaction() != 0)
		return -1;

	int32_t addr = PIOS_FLASHFS_Compact_GetObjAddress(objId, instId);

	// Object currently not saved
	if (addr < 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -1;
	}

	struct fileHeader header;

	// Load file header
	// This information IS redundant with the object table id.  Oh well.  Better safe than sorry.
	if (PIOS_Flash_Internal_ReadData(addr, (uint8_t *) &header, sizeof(header)) != 0) {
		PIOS_Flash_Internal_EndTransaction();
		return -1;
	}

	//overwrite file by 0000
	static const uint16_t clear = 0x0000;
	for (uint32_t iByte = 0; iByte < header.size; iByte += 2)
		if (PIOS_Flash_Internal_WriteData(addr + iByte, (const uint8_t*)&clear, sizeof(clear)) != 0) {
			PIOS_Flash_Internal_EndTransaction();
			return -2;
		}

	if (PIOS_Flash_Internal_EndTransaction() != 0)
		return -1;

	return 0;
}

#endif
