/**
 ******************************************************************************
 *
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FLASH Flash device handler
 * @{
 *
 * @file       pios_flash_w25x.h
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

struct pios_flash_internal_cfg {
	uint32_t addr_start;
	uint32_t addr_end;
	uint32_t sector_size;
};

struct pios_flash_chunk {
	uint8_t * addr;
	uint32_t len;
};

int32_t PIOS_Flash_Internal_Init(const struct pios_flash_internal_cfg * cfg);
int32_t PIOS_Flash_Internal_EraseSector(uint32_t addr);
int32_t PIOS_Flash_Internal_WriteData(uint32_t addr, const uint8_t * data, uint16_t len);
int32_t PIOS_Flash_Internal_ReadData(uint32_t addr, uint8_t * data, uint16_t len);
int32_t PIOS_Flash_Internal_WriteChunks(uint32_t addr, const struct pios_flash_chunk * p_chunk, uint32_t num);
int32_t PIOS_Flash_Internal_StartTransaction();
int32_t PIOS_Flash_Internal_EndTransaction();

