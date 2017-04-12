//*****************************************************************************
//
// flashStore.c - Data logger module to handle storage in flash.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.0.12573 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/flash.h"
#include "flashStore.h"

//*****************************************************************************
//
// This module manages the storage of data logger data into flash memory.
//
//*****************************************************************************

//*****************************************************************************
//
// The next address in flash, that will be used for storing a data record.
//
//*****************************************************************************
static uint32_t g_ui32StoreAddr;

//*****************************************************************************
//
// A buffer used to assemble a complete record of data prior to storing it
// in the flash.
//
//*****************************************************************************
static uint32_t g_pui32RecordBuf[25];

//*****************************************************************************
//
// Char (un)packing in/from UINT32
//
//*****************************************************************************
uint32_t
pack(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3) {
    return (c0 << 24) | (c1 << 16) | (c2 << 8) | c3;
};

uint8_t
unpack_c0(uint32_t p)
{
    return p >> 24;
}

uint8_t
unpack_c1(uint32_t p)
{
    return p >> 16;
}

uint8_t
unpack_c2(uint32_t p)
{
    return p >> 8;
}

uint8_t
unpack_c3(uint32_t p)
{
    return p;
}

//*****************************************************************************
//
// Initializes the flash storage. This is a stub because there is nothing
// special to do.
//
//*****************************************************************************
int32_t
flashStoreInit(void)
{
  // Clear the flash
  flashStoreErase();
  
  return flashStoreNewLogFile(0x0, FLASHSTORE_INIT_OVERWRITE);
}

//*****************************************************************************
//
// This is called at the start of logging to prepare space in flash for
// storage of logged data.  It searches for the first blank area in the
// flash storage to be used for storing records.
//
// If a starting address is specified then the search is skipped and it goes
// directly to the new address.  If the starting address is 0, then it performs
// the search.
//
//*****************************************************************************
int32_t
flashStoreNewLogFile(uint32_t ui32StartAddr, int overwrite)
{
    uint32_t ui32Addr;

    //
    // If a valid starting address is specified, then just use that and skip
    // the search below.
    //
    if((ui32StartAddr >= FLASH_STORE_START_ADDR) &&
       (ui32StartAddr < FLASH_STORE_END_ADDR))
    {
        g_ui32StoreAddr = ui32StartAddr;
        return 1;
    }

    //
    // Start at beginning of flash storage area
    //
    ui32Addr = FLASH_STORE_START_ADDR;

    //
    // Search until a blank is found or the end of flash storage area
    //
    while((HWREG(ui32Addr) != FLASH_STORE_EMPTY_DATA) && (ui32Addr < FLASH_STORE_END_ADDR))
    {
        //
        // If a record signature is found, then increment to the next record
        //
        if((HWREG(ui32Addr) & 0xFFFFFF00) == FLASH_STORE_RECORD_HEADER)
        {
            ui32Addr += HWREG(ui32Addr) & 0xFF;
        }
        else
        {
            //
            // Just advance to the next location in flash
            //
            ui32Addr += FLASH_STORE_BLOCK_WRITE_SIZE;
        }
    }

    //
    // If we are at the end of flash that means no blank area was found.
    // If overwrite disabled, fail. Otherwise, reset to the beginning and erase the first page.
    //

    if(ui32Addr >= FLASH_STORE_END_ADDR)
    {
      if (!overwrite) return (0);
      ui32Addr = FLASH_STORE_START_ADDR;
      FlashErase(ui32Addr);
    }

    //
    // When we reach here we either found a blank location, or made a new
    // blank location by erasing the first page.
    // To keep things simple we are making an assumption that the flash store
    // is not corrupted and that the first blank location implies the start
    // of a blank area suitable for storing data records.
    //
    g_ui32StoreAddr = ui32Addr;

    //
    // Return success indication to caller
    //
    return(1);
}

//*****************************************************************************
//
// This is called each time there is a new data record to log to the flash
// storage area.  A simple algorithm is used which rotates programming
// data log records through an area of flash.  It is assumed that the current
// page is blank.  Records are stored on the current page until a page
// boundary is crossed.  If the page boundary is crossed and the new page
// is not blank (testing only the first location), then the new page is
// erased.  Finally the entire record is programmed into flash and the
// storage pointers are updated.
//
// While storing and when crossing to a new page, if the flash page is not
// blank it is erased.  So this algorithm overwrites old data.
//
// The data is stored in flash as a record, with a flash header prepended,
// and with the record length padded to be a multiple of 4 bytes.  The flash
// header is a 3-byte magic number and one byte of record length.
//
// WARNING: THERE IS A MAXIMUM SIZE OF 255 CHARACTERS
//
//*****************************************************************************
int32_t
flashStoreWriteRecord(uint8_t *record, int size)
{
  if (g_ui32StoreAddr >= FLASH_STORE_END_ADDR)
  {
	  return(0);
  }

  uint32_t ui32Idx, *pui32Record, i;
  //
  // Convert the count to bytes, be sure to pad to 32-bit alignment.
  //
  uint32_t recordSize = size + FLASH_STORE_BLOCK_WRITE_SIZE + (FLASH_STORE_BLOCK_WRITE_SIZE - (size % FLASH_STORE_BLOCK_WRITE_SIZE)) % FLASH_STORE_BLOCK_WRITE_SIZE;

  //
  // Create the flash record header, which is a 3-byte signature and a
  // one byte count of bytes in the record.  Save it at the beginning
  // of the write buffer.
  //
  ui32Idx = FLASH_STORE_RECORD_HEADER | ((recordSize) & 0xFF);
  g_pui32RecordBuf[0] = ui32Idx;

  //
  // Pack chars into record buffer
  //
  pui32Record = (uint32_t *)record;
  for (i=0; i < (size / 4); i++)
  {

    g_pui32RecordBuf[i+1] = pui32Record[i];
  }

  pui32Record = g_pui32RecordBuf;

  //
  // Now program the remaining part of the record (if we crossed a page
  // boundary above) or the full record to the current location in flash
  //
  FlashProgram(pui32Record, g_ui32StoreAddr, recordSize);

  //
  // Increment the storage address to the next location.
  //
  g_ui32StoreAddr += (recordSize);

  //
  // Return number of bytes written to the caller.
  //
  return recordSize;
}

//*****************************************************************************
//
// Erase the data storage area of flash.
//
//*****************************************************************************
void
flashStoreErase(void)
{
    uint32_t ui32Addr;

    //
    // Loop through entire storage area and erase each page.
    //
    for(ui32Addr = FLASH_STORE_START_ADDR; ui32Addr < FLASH_STORE_END_ADDR;
        ui32Addr += FLASH_STORE_BLOCK_ERASE_SIZE)
    {
      if( !IsBlockFree(ui32Addr) )
        FlashErase(ui32Addr);
    }

}

//*****************************************************************************
//
// Determine if the flash block that contains the address is blank.
//
//*****************************************************************************
static int32_t
IsBlockFree(uint32_t ui32BaseAddr)
{
    uint32_t ui32Addr;

    //
    // Make sure we start at the beginning of a 1K block
    //
    ui32BaseAddr &= ~0x3FF;

    //
    // Loop through every address in this block and test if it is blank.
    //
    for(ui32Addr = 0; ui32Addr < FLASH_STORE_BLOCK_ERASE_SIZE; ui32Addr += FLASH_STORE_BLOCK_WRITE_SIZE)
    {
        if(HWREG(ui32BaseAddr + ui32Addr) != FLASH_STORE_EMPTY_DATA)
        {
            //
            // Found a non-blank location, so return indication that block
            // is not free.
            //
            return 0;
        }
    }

    //
    // If we made it to here then every location in this block is erased,
    // so return indication that the block is free.
    //
    return 1;
}

//*****************************************************************************
//
// Report to the user the amount of free space in the data storage area.
//
//*****************************************************************************
int32_t
flashStoreFree(void)
{
    uint32_t ui32Addr, ui32FreeBlocks = 0;

    //
    // Loop through each block of the storage area and count how many blocks
    // are free and non-free.
    //
    for(ui32Addr = FLASH_STORE_START_ADDR; ui32Addr < FLASH_STORE_END_ADDR;
        ui32Addr += FLASH_STORE_BLOCK_ERASE_SIZE)
    {
        if(IsBlockFree(ui32Addr))
        {
            ui32FreeBlocks++;
        }
    }

    return ui32FreeBlocks;
}

//*****************************************************************************
//
// Report to the user the amount of used space in the data storage area.
//
//*****************************************************************************
int32_t
flashStoreUsed(void)
{
    uint32_t ui32Addr, ui32UsedBlocks = 0;

    //
    // Loop through each block of the storage area and count how many blocks
    // are free and non-free.
    //
    for(ui32Addr = FLASH_STORE_START_ADDR; ui32Addr < FLASH_STORE_END_ADDR;
        ui32Addr += FLASH_STORE_BLOCK_ERASE_SIZE)
    {
        if(!IsBlockFree(ui32Addr))
        {
            ui32UsedBlocks++;
        }
    }

    return ui32UsedBlocks;
}


//*****************************************************************************
//
// Return the current address being used for storing records.
//
//*****************************************************************************
uint32_t
flashStoreGetCurrentAddr(void)
{
    return g_ui32StoreAddr;
}

//*****************************************************************************
//
// Set the current address being used for storing records.
//
//*****************************************************************************
void
flashStoreSetCurrentAddr(uint32_t newAddr)
{
  g_ui32StoreAddr = newAddr;
}

//*****************************************************************************
//
// Return the data in memory at the passed in address
//
//*****************************************************************************
uint32_t
flashStoreGetData(uint32_t Addr)
{
  return HWREG(Addr);
}