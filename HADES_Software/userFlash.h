//*****************************************************************************
//
// flash.h - on-chip flash interface software for HADES Payload.
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

#ifndef FLASH
#define FLASH

//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include "HADES_types.h"

//*****************************************************************************
//
// Project Definitions
//
//*****************************************************************************
#define BYTES_OF_DATA           0x32000 // 200 kB         
#define DATA_STORAGE_SZ         (NUM_DATA_PTS*sizeof(sensorData_t))

#define FLASH_BLOCK_SZ          SysCtlFlashSectorSizeGet()

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
void ConfigureFlash(void);

void flash_storeDataPoint(dynamicsData_t *dynamicsData, atmosData_t *atmosData);

void flash_outputData(void); //Should only be called if UART is enabled (protect with #ifdef USE_UART)

#endif