//*****************************************************************************
//
// microSD.c - microSD card interface softwaer for HADES Payload.
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

//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include "microSD.h"

#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"

#include "driverlib/timer.h"

// FatFS files
#include "fatfs/diskio.h"
#include "fatfs/ff.h"

// Ryan Claus User Files
#include "userlib/timer.h"

#define MAX_MOUNT_ATTEMPTS      100
#define MAX_FOPEN_ATTEMPTS      100

//*****************************************************************************
//
// Local Function Prototypes
//
//*****************************************************************************
void FatFSTimerIntHandler(void);

//*****************************************************************************
//
// Necessary Global Variables
//
//*****************************************************************************
FATFS *g_psFlashMount;
FIL *g_psFlashFile;

//*****************************************************************************
//
// Function Definitions
//
//*****************************************************************************
// Initialize the microSD card access
uint_fast8_t SDCardInit( void ) 
{
  char pcFilename[250];
  uint_fast8_t vui8Index = 0;
  int idx = 0;
  
  // Mount the File System
  while (f_mount(g_psFlashMount, "", 1) != FR_OK && idx <= MAX_MOUNT_ATTEMPTS)
  { idx++;  }
  
  if(idx == MAX_MOUNT_ATTEMPTS) // Max Mount Attempts reached, error out
    return(ERROR);
  
  // Create the filename
  sprintf(pcFilename, "%s.%s", SDCARD_FILENAME, SDCARD_EXT);
  
  // if file already exists, append nuumber to it after last number
  for (vui8Index = 1; f_stat(pcFilename, NULL) != FR_NO_FILE && vui8Index <= 99; vui8Index++) 
  {
    sprintf(pcFilename, "%s%d.%s", SDCARD_FILENAME, vui8Index, SDCARD_EXT);
  }
  
  // Ensure the file is not present (delete it)
  f_unlink(pcFilename);
  
  // Open the file
  idx = 0;
  while (f_open(g_psFlashFile, pcFilename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK && idx <= MAX_FOPEN_ATTEMPTS)
  { idx++;  }
  
  if(idx == MAX_FOPEN_ATTEMPTS) // Max Mount Attempts reached, error out
    return(ERROR);

  // Put a comment into the top of the file
  if (f_puts("HADES\n"SDCARD_COMMENT"\n", g_psFlashFile) == EOF) {
    //
    // Out of flash memory
    //
    f_close(g_psFlashFile);
    f_mount(NULL, "", 0);
    return (ERROR);
  } else 
  {
    f_sync(g_psFlashFile);
    return (OK);
  }
}

// Write a string of arbitrary length to the specified file
uint_fast8_t SDCardWrite(char * writeString, FIL * file)
{
  uint8_t rv = ERROR;
  
  if (f_puts(writeString, file) == EOF) 
    SDCardInit();
  else
    rv = OK;
  
  f_sync(g_psFlashFile);
  
  return rv;
}
  