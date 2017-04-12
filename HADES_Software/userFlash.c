//*****************************************************************************
//
// flash.c - on-chip flash interface software for HADES Payload.
//
// This software is to be used on the Payloads of the HADES project by Team
// Ascension. The software contained is to utilize the on-chip flash for sensor
// data storage.
//
// Primary Author: Matthew Demi Vis
//                 vism@my.erau.edu
// Others Responsible: Jason Lathbury        Garrett Trahern
//                    lathburj1@my.erau.edu  traherng@my.erau.edu
//
//*****************************************************************************

//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "driverlib/sysctl.h"

#include "flashStore.h"

#include "driverlib/flash.h"
#include "inc/hw_flash.h"

#include "userFlash.h"

//*****************************************************************************
//
// Project Definitions
//
//*****************************************************************************
#define FLASH_PB_START_ADDR     ( 0x0003ffff - (2*FLASH_BLOCK_SZ) ) // 2 blocks before end of FLASH
#define FLASH_PB_END_ADDR       ( 0x0003ffff ) // end of FLASH from .icf linker file
#define FLASH_PB_BLOCK_SZ       (64)

#define NEXT_FLASH_ADDR         (flashBaseAddr + (flashBlockIdx*FLASH_BLOCK_SZ) + flashWriteIdx)

//*****************************************************************************
//
// External functions used from main.c
//
//*****************************************************************************
extern void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line, char * msg);
extern int makeDataString(char *outString, dynamicsData_t *dynamicsData, atmosData_t *atmosData);
extern void UARTSend(const char *pucBuffer);

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
// to make sure room is reserved in the flash
#pragma data_alignment = 1024
__root const uint8_t flashData[BYTES_OF_DATA] @ ".storage";

uint32_t flashBytesWritten;

// Flash base address to offset (flashWriteIdx) from
uint32_t flashBaseAddr;

bool FreeSpaceAvailable;
int32_t blocksFree;

// End of data chunk to indicate end of data
uint32_t end_of_data = 0xFFFFFFFF;
#define END_OF_DATA_SZ  (32/8)

//*****************************************************************************
//
// Initialize the Flash interface
//
//*****************************************************************************
void ConfigureFlash(void)
{
  flashBytesWritten = 0;
  blocksFree = flashStoreFree();
  FreeSpaceAvailable = flashStoreInit();
}

//*****************************************************************************
//
// Store the latest data point in the Flash
//
//*****************************************************************************
void flash_storeDataPoint(dynamicsData_t *dynamicsData, atmosData_t *atmosData)
{
	// Check to make sure there is still data available
	
	
	// Don't store the atmospheric data
	if(atmosData == NULL)
	{
		flashStoreWriteRecord((uint8_t *)dynamicsData, DYNAMICS_STRUCT_SZ);
		FreeSpaceAvailable = flashStoreWriteRecord((uint8_t *)&end_of_data, END_OF_DATA_SZ);
                flashBytesWritten += DYNAMICS_STRUCT_SZ + END_OF_DATA_SZ;
	}
	else
	{
		flashStoreWriteRecord((uint8_t *)dynamicsData, DYNAMICS_STRUCT_SZ);
		flashStoreWriteRecord((uint8_t *)atmosData, ATMOS_STRUCT_SZ);
		FreeSpaceAvailable = flashStoreWriteRecord((uint8_t *)&end_of_data, END_OF_DATA_SZ);
                flashBytesWritten += DYNAMICS_STRUCT_SZ + ATMOS_STRUCT_SZ + END_OF_DATA_SZ;
	}
}

//*****************************************************************************
//
// Output all the data stored in this round out to the UART. Should only be called
// if UART is enabled (protect with #ifdef USE_UART)
//
//*****************************************************************************
void flash_outputData(void)
{
  uint32_t tempData[(DYNAMICS_STRUCT_SZ + ATMOS_STRUCT_SZ)/4 + 1];
  uint32_t idx = 0;
  
  assert(FLASH_STORE_START_ADDR + flashBytesWritten < FLASH_STORE_END_ADDR);
  
  // reset the count for the data output
  makeDataString(NULL, NULL, NULL);
  
  // Send column headers
  UARTSend("  # , AccelX, AccelY, AccelZ, GyroX , GyroY , GyroZ , Mag X , Mag Y , Mag Z , Press ,  Temp ,   Alt  \n\r");
  UARTSend("    , m/s^2 , m/s^2 , m/s^2 , rad/s , rad/s , rad/s ,   uT  ,   uT  ,   uT  ,  inHg ,   C   ,    m   \n\r");

  
  for(uint32_t addr = FLASH_STORE_START_ADDR; addr <= FLASH_STORE_START_ADDR + flashBytesWritten; addr += 0x4)
  {
    uint32_t readByte = flashStoreGetData(addr);
    
    if(readByte == end_of_data)
    {
      char outString[512];
      
      if(idx > DYNAMICS_STRUCT_SZ) // have atmos data also
      {
        makeDataString(outString, (dynamicsData_t *)tempData, (atmosData_t *)&tempData[DYNAMICS_STRUCT_SZ/4]);
      }
      else
      {
        makeDataString(outString, (dynamicsData_t *)tempData, NULL);
      }
      UARTSend(outString);
      idx = 0;
    }
    else if(readByte == FLASH_STORE_RECORD_HEADER)
    { /* just skip this byte */ }
    else
    {
      tempData[idx] = readByte;
      idx++;
    }
  }
}