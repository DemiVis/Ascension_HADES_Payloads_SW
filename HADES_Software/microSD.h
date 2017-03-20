//*****************************************************************************
//
// microSD.h - microSD card interface softwaer for HADES Payload.
//
// This software is to be used on the Payloads of the HADES project by Team
// Ascension. The software contained is to allow interface between the TI
// LaunchPad and a properly connected microSD card for data storage.
//
// Primary Author: Matthew Demi Vis
//                 vism@my.erau.edu
// Others Responsible: Jason Lathbury        Garrett Trahern
//                    lathburj1@my.erau.edu  traherng@my.erau.edu
//
//*****************************************************************************

#ifndef MICROSD
#define MICROSD

//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include <stdint.h>

#include "fatfs/ff.h"

//*****************************************************************************
//
// Project Definitions
//
//*****************************************************************************
#define SDCARD_ENABLE      true

#define SDCARD_FILENAME 			"HADES_data"
#define SDCARD_EXT      			"csv"
#define SDCARD_COMMENT 				"Raw data read from sensors"
#define SDCARD_TIMERPROC_TIMER_BASE		TIMER0_BASE
#define SDCARD_TIMERPROC_RATE 			100 		// in hz

#define OK                                      1
#define ERROR                                   0


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

uint_fast8_t SDCardInit(void);
uint_fast8_t SDCardWrite(char * writeString, FIL * file);

#endif