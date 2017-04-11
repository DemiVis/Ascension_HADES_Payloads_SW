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

#include "utils/flash_pb.h"

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
__root const sensorData_t flashData[NUM_DATA_PTS] @ ".storage";

// Global index into the flash storage block. 
// indexes flashWriteIdx bytes into block # flashBlockIdx after &flashData.
uint32_t flashBlockIdx;
uint32_t flashWriteIdx;

// Flash base address to offset (flashWriteIdx) from
uint32_t flashBaseAddr;

// End of data chunk to indicate end of data
uint32_t end_of_data = 0xFFFFFFFF;
#define END_OF_DATA_SZ  (32/8)

//*****************************************************************************
//
// IPrivate functions for use below
//
//*****************************************************************************
bool isBlockErased(uint32_t block_num)
{
  uint8_t erasedBlock[1024];    // Must be same as FLASH_BLOCK_SZ
  
  // TODO:
  memset((void *)erasedBlock, 0xFFFFFF, FLASH_BLOCK_SZ);
  if(memcmp((void *)erasedBlock, (void *)(flashBaseAddr + (flashBlockIdx*FLASH_BLOCK_SZ)), FLASH_BLOCK_SZ) == 0)
    return true;
  else
    return false;
}

//*****************************************************************************
//
// Initialize the Flash interface
//
//*****************************************************************************
void ConfigureFlash(void)
{
  uint8_t flashPB[FLASH_PB_BLOCK_SZ];
  
  // Check that everything is alright before start initilizing
  assert(DYNAMICS_STRUCT_SZ % 4 == 0);
  assert(ATMOS_STRUCT_SZ % 4 == 0);
  
  // Initilize the parameter block
  FlashPBInit(FLASH_PB_START_ADDR, FLASH_PB_END_ADDR, FLASH_PB_BLOCK_SZ);
  
  flashPB[0] = 0; // Sequence Number
  //flashPB[1] = CRC?
  FlashPBSave(flashPB);
  
  flashBaseAddr = (uint32_t)&flashData;
  
  flashBlockIdx = 0;
  flashWriteIdx = 0;
}

//*****************************************************************************
//
// Store the latest data point in the Flash
//
//*****************************************************************************
void flash_storeDataPoint(dynamicsData_t *dynamicsData, atmosData_t *atmosData)
{
  int32_t temp;
  
  IntMasterDisable();
  
  // if it's the beginning of a new block, check if it's erased and if not erase it
  if( flashWriteIdx == 0 && !isBlockErased(flashBlockIdx) );
  {
     temp = FlashErase( flashBaseAddr + (flashBlockIdx*FLASH_BLOCK_SZ) );
     if(temp != 0)
       MPU9150AppErrorHandler(__FILE__, __LINE__, "Flash Block erase failure");
  }
  
  // Write dynamics data
  assert(NEXT_FLASH_ADDR % 4 == 0); // make sure the address is a multiple of 4
  temp = FlashProgram((uint32_t *)dynamicsData, NEXT_FLASH_ADDR, DYNAMICS_STRUCT_SZ);
  if(temp != 0)
    MPU9150AppErrorHandler(__FILE__, __LINE__, "Dynamics Data flash programming failure!");
  flashWriteIdx += DYNAMICS_STRUCT_SZ;

  // If atmoshperic data, write it too
  if(atmosData != NULL)
  {      
    assert(NEXT_FLASH_ADDR % 4 == 0); // make sure the address is a multiple of 4
    temp = FlashProgram((uint32_t *)atmosData, NEXT_FLASH_ADDR, ATMOS_STRUCT_SZ);
    if(temp != 0)
      MPU9150AppErrorHandler(__FILE__, __LINE__, "Atmos Data flash programming failure!");
    flashWriteIdx += ATMOS_STRUCT_SZ;
  }
  
  // Write end of data point flag
  //flashBaseAddr[flashWriteIdx] = END_OF_DATA_FLAG;
  FlashProgram(&end_of_data, NEXT_FLASH_ADDR, END_OF_DATA_SZ);
  flashWriteIdx += END_OF_DATA_SZ;
  
  IntMasterEnable();
}

//*****************************************************************************
//
// Output all the data stored in this round out to the UART. Should only be called
// if UART is enabled (protect with #ifdef USE_UART)
//
//*****************************************************************************
void flash_outputData(void)
{
  // TODO: remake this function!
  
  /*uint32_t tempData[SENSOR_STRUCT_SZ+2];
  char tempStr[254];
  
  // Setup the tempData string with null terminator at the end just in case
  tempData[SENSOR_STRUCT_SZ+1] = 0;
  
  // Reset data string index to zero
  makeDataString(NULL, NULL, NULL);
  
  // Write out all the data
  for(uint32_t idx = 0, jdx = 0; idx <= flashWriteIdx; idx++)
  {    
    if(flashBaseAddr == end_of_data || jdx >= SENSOR_STRUCT_SZ)
    {      
      // If atmospheric data present
      if(jdx > DYNAMICS_STRUCT_SZ)
        // Make the data string from both data streams
        makeDataString(tempStr, (dynamicsData_t *)tempData, (atmosData_t *)(tempData+DYNAMICS_STRUCT_SZ));
      else
        // Make the data string with just dynamics data
        makeDataString(tempStr, (dynamicsData_t *)tempData, NULL);
       
      // Print the string to UART
      UARTSend(tempStr);
      
      // Reset the count for the data string
      jdx = 0;
    }
    else
    {
      // Store this chunk of data locally
      tempData[jdx] = *((uint32_t *)flashBaseAddr);
        
      // Move on in the local buffer
      jdx++;
    }
  }*/
}