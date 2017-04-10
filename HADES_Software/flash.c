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

__root const sensorData_t flashData[4000] @ ".storage";

//*****************************************************************************
//
// Store the latest data point in the Flash
//
//*****************************************************************************
void flash_storeDataPoint(dynamicsData_t *dynamicsData, atmosData_t *atmosData)
{
  
  // TODO: implement this
  if(atmosData == NULL) // No atmoshperic data for this data point
  {
    
  }
  else // atmospheric and dynamics data
  {
    
  }
}

//*****************************************************************************
//
// Output all the data stored in this round out to the UART. Should only be called
// if UART is enabled (protect with #ifdef USE_UART)
//
//*****************************************************************************
void OutputFlashData(void)
{
	// TODO: Implement this
}