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

#include "flash.h"

// External error handler for sending error messages
extern void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line, char * msg);

// TODO: Move these functions to "helper funtions" file separately and include in .h
extern int makeDataString(char *outString, dynamicsData_t *dynamicsData, atmosData_t *atmosData);
extern void UARTSend(const char *pucBuffer);

// to make sure room is reserved in the flash
__root const sensorData_t flashData[NUM_DATA_PTS] @ ".storage";

// Global index into the flash storage block. indexes in bytes.
uint32_t flashWriteIdx;

// Flash base address to offset (flashWriteIdx) from
uint8_t *flashBaseAddr;

// defines to make this more readable
#define FLASH_PB_START_ADDR     ( 0x0003ffff - (2*FLASH_BLOCK_SZ) ) // 2 blocks before end of FLASH
#define FLASH_PB_END_ADDR       ( 0x0003ffff ) // end of FLASH from .icf linker file
#define FLASH_PB_BLOCK_SZ       (64)

#define END_OF_DATA_FLAG        255

//*****************************************************************************
//
// Initialize the Flash interface
//
//*****************************************************************************
void ConfigureFlash(void)
{
  uint8_t flashPB[FLASH_PB_BLOCK_SZ];
  
  // Effectively Asserts to make sure all of the #defined constants are right
  // see SW-TM4C-UTILS-UG...pdf pg 18 for more details
  if( (FLASH_PB_END_ADDR - FLASH_PB_START_ADDR) < (2 * FLASH_BLOCK_SZ) )
    MPU9150AppErrorHandler(__FILE__, __LINE__, "Flash Init Assert Break #1!");
  if( (FLASH_BLOCK_SZ % FLASH_PB_BLOCK_SZ) != 0 )
    MPU9150AppErrorHandler(__FILE__, __LINE__, "Flash Init Assert Break #2!");
  /*if( (FLASH_PB_END_ADDR - FLASH_PB_START_ADDR)/FLASH_PB_BLOCK_SZ > 128 )
    MPU9150AppErrorHandler(__FILE__, __LINE__, "Flash Init Assert Break #3!");*/
  
  // Initilize the parameter block
  FlashPBInit(FLASH_PB_START_ADDR, FLASH_PB_END_ADDR, FLASH_PB_BLOCK_SZ);
  
  flashPB[0] = 0; // Sequence Number
  //flashPB[1] = CRC?
  FlashPBSave(flashPB);
  
  flashBaseAddr = (uint8_t *)&flashData;
  
  flashWriteIdx = 0;
}

//*****************************************************************************
//
// Store the latest data point in the Flash
//
//*****************************************************************************
void flash_storeDataPoint(dynamicsData_t *dynamicsData, atmosData_t *atmosData)
{
  // Write dynamics data
  memcpy((void *)flashBaseAddr[flashWriteIdx], (void *)dynamicsData, DYNAMICS_STRUCT_SZ);
  flashWriteIdx += DYNAMICS_STRUCT_SZ;

  // If atmoshperic data, write it too
  if(atmosData != NULL)
  {      
    memcpy((void *)flashBaseAddr[flashWriteIdx], (void *)atmosData, ATMOS_STRUCT_SZ);
    flashWriteIdx += ATMOS_STRUCT_SZ;
  }
  
  // Write end of data point flag
  flashBaseAddr[flashWriteIdx] = END_OF_DATA_FLAG;
  flashWriteIdx++;
}

//*****************************************************************************
//
// Output all the data stored in this round out to the UART. Should only be called
// if UART is enabled (protect with #ifdef USE_UART)
//
//*****************************************************************************
void flash_outputData(void)
{
  uint8_t tempData[SENSOR_STRUCT_SZ+2];
  char tempStr[254];
  
  // Setup the tempData string with null terminator at the end just in case
  tempData[SENSOR_STRUCT_SZ+1] = 0;
  
  // Reset data string index to zero
  makeDataString(NULL, NULL, NULL);
  
  // Write out all the data
  for(uint32_t idx = 0, jdx = 0; idx <= flashWriteIdx; idx++)
  {    
    if(flashBaseAddr[idx] == END_OF_DATA_FLAG || jdx >= SENSOR_STRUCT_SZ)
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
      // Store this byte of data locally
      tempData[jdx] = flashBaseAddr[idx];
        
      // Move on in the local buffer
      jdx++;
    }
  }
}