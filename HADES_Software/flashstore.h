//*****************************************************************************
//
// flashStore.h - Prototypes and definitions for flash storage module.
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
#ifndef __flashStore_H__
#define __flashStore_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Define the beginning and end of the flash storage area.  You must make sure
// that this area is well clear of any space occupied by the application
// binary, and that this space is not used for any other purpose.
// The start and end addresses must be 1K aligned.  The end address is
// exclusive - it is 1 value greater than the last valid location used for
// storage.
//
//*****************************************************************************
#define FLASH_STORE_START_ADDR  0x00400
#define FLASH_STORE_END_ADDR    0x32400

//*****************************************************************************
//
// Flash block write size
//
//*****************************************************************************
#define FLASH_STORE_BLOCK_WRITE_SIZE 0x04

//*****************************************************************************
//
// Flash block erase size
//
//*****************************************************************************
#define FLASH_STORE_BLOCK_ERASE_SIZE 0x400

//*****************************************************************************
//
// Empty data (all ones)
//
//*****************************************************************************
#define FLASH_STORE_EMPTY_DATA 0xFFFFFFFF

//*****************************************************************************
//
// Define the flash record header, which is a 3-byte signature and an added
// one byte count of bytes in the record.  Saved at the beginning
// of the write buffer.
//
//*****************************************************************************
#define FLASH_STORE_RECORD_HEADER 0x53554100
#define FLASH_STORE_RECORD_HEADER_SZ 4 // bytes 

//*****************************************************************************
//
// Configuration of whether or not to overwrite on initialization
//
//*****************************************************************************
#define FLASHSTORE_INIT_OVERWRITE 1

//*****************************************************************************
//
// Module function prototypes.
//
//*****************************************************************************
extern uint32_t pack(uint8_t, uint8_t, uint8_t, uint8_t);
extern uint8_t  unpack_c0(uint32_t);
extern uint8_t  unpack_c1(uint32_t);
extern uint8_t  unpack_c2(uint32_t);
extern uint8_t  unpack_c3(uint32_t);
extern int32_t  flashStoreInit(void);
extern int32_t  flashStoreNewLogFile(uint32_t ui32StartAddr, int);
extern int32_t  flashStoreWriteRecord(uint8_t *, int);
extern int32_t  flashStoreSave(void);
extern void     flashStoreErase(void);
static int32_t  IsBlockFree(uint32_t ui32BaseAddr);
extern int32_t  flashStoreFree(void);
extern int32_t  flashStoreUsed(void);
extern uint32_t flashStoreGetData(uint32_t);
extern uint32_t flashStoreGetCurrentAddr(void);
extern void     flashStoreSetCurrentAddr(uint32_t);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __flashStore_H__