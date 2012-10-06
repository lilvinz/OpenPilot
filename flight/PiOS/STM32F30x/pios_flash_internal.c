/**
 ******************************************************************************
 *
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FLASH Flash device handler
 * @{
 *
 * @file       pios_flash_w25x.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Driver for talking to W25X flash chip (and most JEDEC chips)
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
#include "pios.h"



enum pios_flash_internal_dev_magic {
	PIOS_FLASH_INTERNAL_DEV_MAGIC = 0x1e6bc239,
};

//! Device handle structure
struct internal_flash_dev {
	const struct pios_flash_internal_cfg * cfg;
#if defined(PIOS_INCLUDE_FREERTOS)
	xSemaphoreHandle transaction_lock;
#endif
	enum pios_flash_internal_dev_magic magic;
};

//! Global structure for this flash device
struct internal_flash_dev * flash_dev;

//! Private functions
static int32_t PIOS_Flash_Internal_Validate(struct internal_flash_dev * dev);
static struct internal_flash_dev * PIOS_Flash_Internal_alloc(void);

/**
 * @brief Allocate a new device
 */
static struct internal_flash_dev * PIOS_Flash_Internal_alloc(void)
{
	struct internal_flash_dev * flash_dev;
	
	flash_dev = (struct internal_flash_dev *)pvPortMalloc(sizeof(*flash_dev));
	if (!flash_dev) return (NULL);
	
	flash_dev->magic = PIOS_FLASH_INTERNAL_DEV_MAGIC;
#if defined(PIOS_INCLUDE_FREERTOS)
	flash_dev->transaction_lock = xSemaphoreCreateMutex();
#endif
	return(flash_dev);
}

/**
 * @brief Validate the handle to the spi device
 */
static int32_t PIOS_Flash_Internal_Validate(struct internal_flash_dev * dev) {
	if (dev == NULL) 
		return -1;
	if (dev->magic != PIOS_FLASH_INTERNAL_DEV_MAGIC)
		return -2;
	return 0;
}

/**
 * @brief Initialize the flash device
 */
int32_t PIOS_Flash_Internal_Init(const struct pios_flash_internal_cfg * cfg)
{
	flash_dev = PIOS_Flash_Internal_alloc();
	if (flash_dev == NULL)
		return -1;

	flash_dev->cfg = cfg;

	//Enable register access
	FLASH_Unlock();

	return 0;
}

/**
 * @brief Grab the semaphore to perform a transaction
 * @return 0 for success, -1 for timeout
 */
int32_t PIOS_Flash_Internal_StartTransaction()
{
#if defined(PIOS_INCLUDE_FREERTOS)
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;

	if (xSemaphoreTake(flash_dev->transaction_lock, portMAX_DELAY) != pdTRUE)
		return -1;
#endif
	return 0;
}

/**
 * @brief Release the semaphore to perform a transaction
 * @return 0 for success, -1 for timeout
 */
int32_t PIOS_Flash_Internal_EndTransaction()
{
#if defined(PIOS_INCLUDE_FREERTOS)
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;

	if (xSemaphoreGive(flash_dev->transaction_lock) != pdTRUE)
		return -1;
#endif
	return 0;
}

/**
 * @brief Erase a sector on the flash chip
 * @param[in] add Address of flash to erase
 * @returns 0 if successful
 * @retval -1 on error
 * @retval
 */
int32_t PIOS_Flash_Internal_EraseSector(uint32_t addr)
{
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;

	FLASH_Status ret = FLASH_ErasePage(addr);
	if (ret != FLASH_COMPLETE)
		return -1;

	return 0;
}

/**
 * @brief Write data
 * @param[in] addr Address in flash to write to
 * @param[in] data Pointer to data to write to flash
 * @param[in] len Length of data to write (must be a multipe of 2)
 * @return Zero if success or error code
 * @retval -1 operation failed
 */
int32_t PIOS_Flash_Internal_WriteData(uint32_t addr, const uint8_t * data, uint16_t len) __attribute((optimize(0)));
int32_t PIOS_Flash_Internal_WriteData(uint32_t addr, const uint8_t * data, uint16_t len)
{
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;

	//handle first byte if odd address
	if ((addr & 1) != 0)
	{
		uint32_t first_addr = addr & (~1);
		uint16_t first_data;

		if (PIOS_Flash_Internal_ReadData(first_addr, (uint8_t*)&first_data, sizeof(first_data)) != 0)
			return -1;

		first_data &= 0xff00;
		first_data |= data[0];

		FLASH_Status ret = FLASH_ProgramHalfWord(first_addr, first_data);
		if (ret != FLASH_COMPLETE)
			return -1;

		++addr;
		++data;
		--len;
	}

	for (uint32_t iByte = 0; iByte < (len & (~1)); iByte += 2) {
		uint16_t halfword = (data[iByte + 1] << 8) | (data[iByte]);
		FLASH_Status ret = FLASH_ProgramHalfWord(addr + iByte, halfword);
		if (ret != FLASH_COMPLETE)
			return -1;
	}

	//handle last byte if odd count after handling of first byte
	if ((len & 1) != 0)
	{
		uint32_t last_addr = (addr + len - 1) & (~1);
		uint16_t last_data;

		if (PIOS_Flash_Internal_ReadData(last_addr, (uint8_t*)&last_data, sizeof(last_data)) != 0)
			return -1;

		last_data &= 0x00ff;
		last_data |= data[len - 1] << 8;

		FLASH_Status ret = FLASH_ProgramHalfWord(last_addr, last_data);
		if (ret != FLASH_COMPLETE)
			return -1;
	}

	return 0;
}

/**
 * @brief Write multiple chunks of data in one transaction
 * @param[in] addr Address in flash to write to
 * @param[in] p_chunk Pointer to array of chunks
 * @param[in] num Number of chunks in array
 * @return Zero if success or error code
 * @retval -1 operation failed
 */
int32_t PIOS_Flash_Internal_WriteChunks(uint32_t addr, const struct pios_flash_chunk * p_chunk, uint32_t num)
{
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;
	
	for (uint32_t i = 0; i < num; i++) {
		const struct pios_flash_chunk * chunk = &p_chunk[i];

		int32_t retval = PIOS_Flash_Internal_WriteData(addr, chunk->addr, chunk->len);
		if (retval != 0)
			return retval;

		addr += chunk->len;
	}

	return 0;
}

/**
 * @brief Read data from a location in flash memory
 * @param[in] addr Address in flash to write to
 * @param[in] data Pointer to data to write from flash
 * @param[in] len Length of data to write (max 256 bytes)
 * @return Zero if success or error code
 * @retval -1 Unable to claim SPI bus
 */
int32_t PIOS_Flash_Internal_ReadData(uint32_t addr, uint8_t * data, uint16_t len)
{
	if (PIOS_Flash_Internal_Validate(flash_dev) != 0)
		return -1;

	memcpy(data, (void*)addr, len);

	return 0;
}
