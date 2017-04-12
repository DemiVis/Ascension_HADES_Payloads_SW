//*****************************************************************************\
//*****************************************************************************
//
// main.c - Main Program for HADES Payload Software.
//
// This software is to be used on the Payloads of the HADES project by Team
// Ascension. Each payload will run this software on an On-Board Computer (OBC)
// The OBC this software is designed for is a TI LaunchPad (TM4C123GXL) along
// with a TI SensorHub (BOOSTXL-SENSHUB) for sensor integration. For primary
// data storage a breakout interface borad should be used to connect a microSD
// card to the SPI lines specified in microSD.h.
//
// This software is based off of the TI example project 'compdcm_mpu9150'.
//
// For debugging purposes connect a serial terminal program to the LaunchPad's 
// ICDI virtual serial port at 115,200 baud. Use eight bits per byte, no parity
// and one stop bit.
//
// Primary Author: Matthew Demi Vis
//                 vism@my.erau.edu
// Others Responsible: Jason Lathbury        Garrett Trahern
//                    lathburj1@my.erau.edu  traherng@my.erau.edu
//
//*****************************************************************************
//*****************************************************************************

// Basic Configurations
#define USE_SDCARD              0    // Set to 1 to use the SD card for storage
#define USE_UART                1    // Set to 1 to use the UART for data output
#define USE_FLASH               (!USE_SDCARD) // If not using the SD card, use the flash to store all the data


//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/bmp180.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"

#include "drivers/buttons.h"
#include "drivers/rgb.h"

#include "HADES_types.h"

#ifdef USE_SDCARD
#include "microSD.h"
#endif

#ifdef USE_FLASH
#include "userFlash.h"
#endif

//*****************************************************************************
//
// Project Definitions
//
//*****************************************************************************
#define BMP180_I2C_ADDRESS      0x77
#define MPU9150_I2C_ADDRESS     0x68
#define PRINT_SKIP_COUNT        10

#define XAXIS                   0
#define YAXIS                   1
#define ZAXIS                   2

#define MAX_DATASTR_LEN         1024

#define SENSHUB_LED_PORT        GPIO_PORTD_BASE
#define SENSHUB_LED_PIN         GPIO_PIN_2

#define ATMOS_TIMER_SYSCTL      SYSCTL_PERIPH_TIMER2
#define ATMOS_TIMER_BASE        TIMER2_BASE
#define ATMOS_SUBTIMER          TIMER_A
#ifdef USE_SDCARD
#define ATMOS_FREQ              40       // Hz
#else   // using flash
#define ATMOS_FREQ              1       // Hz
#endif
#define ATMOS_SKIP_COUNT        4       // DO atmospheric stuff every 4th dynamics stuff

#define DYNAMICS_TIMER_SYSCTL   SYSCTL_PERIPH_TIMER3
#define DYNAMICS_TIMER_BASE     TIMER3_BASE
#define DYNAMICS_SUBTIMER       TIMER_A
#ifdef USE_SDCARD
#define DYNAMICS_FREQ           100       // Hz
#else   // using flash
#define DYNAMICS_FREQ           4       // Hz
#endif

#define GO_DELAY                1.0f // Delay (in sec) for GO button push 
#define BUTTON_HOLD_COUNT	(100000) // Arbitrary number

#if USE_UART
#define GO_UART_CHAR            'G'
#define STOP_UART_CHAR		'S'
#define PROJ_UART_BASE          UART0_BASE              // Which UART to use
#define PROJ_UART_PERIPH        SYSCTL_PERIPH_UART0     // Note if this changes
#define PROJ_UART_GPIO_BASE     GPIO_PORTA_BASE         // ConfigureUART() must
#define PROJ_UART_GPIO_PERIPH   SYSCTL_PERIPH_GPIOA     // must still be updated
#define OUTPUT_CONFIRM_STR	"Output Data"
#endif

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint32_t g_pui32Colors[3];      // for holding the color values for the RGB.

tI2CMInstance g_sI2CInst;       // for the I2C master driver.
tBMP180 g_sBMP180Inst;          // for the BMP180 sensor driver.
tMPU9150 g_sMPU9150Inst;        // for the ISL29023 sensor driver.
tCompDCM g_sCompDCMInst;        // to manage the DCM state.

uint32_t g_ui32PrintSkipCounter;// to control the rate of data to the terminal.

atmosData_t fAtmosData;         // Temp atmos data storage
dynamicsData_t fDynamicsData;   // Temp dynamics data storage

// Flags
volatile uint_fast8_t g_vui8I2CDoneFlag; // flags to alert main that MPU9150 I2C
                                         // transaction is complete
volatile uint_fast8_t g_vui8ErrorFlag;   // flags to alert main that MPU9150 I2C 
                                         // transaction error has occurred.
volatile uint_fast8_t g_vui8BMPDataFlag; // flags to alert main that BMP180
                                         // data is ready.
volatile uint_fast8_t g_vui8MPUDataFlag; // flags to alert main that MPU9150
                                         // data is ready to be retrieved.
volatile bool newAtmosDataReady;         // flag to indicate atmos data ready to be written
volatile bool newDynamicsDataReady;      // flag to indicate dyanmics data ready to be written

// External Variables
#if USE_SDCARD
extern FIL *g_psFlashFile;               // From microSD.c
FIL dataFile;                   // File where the data is stored on the SD card
#endif

//*****************************************************************************
//
// Enable/Disable the timers for sensor (atmos and dynamics) data collection
//
//*****************************************************************************
void DisableSensorTimers(void)
{
    // Disable timers until we're ready to use them
  //TimerDisable(ATMOS_TIMER_BASE, ATMOS_SUBTIMER);
  TimerDisable(DYNAMICS_TIMER_BASE, DYNAMICS_SUBTIMER);
}
void EnableSensorTimers(void)
{
    // Disable timers until we're ready to use them
  //TimerEnable(ATMOS_TIMER_BASE, ATMOS_SUBTIMER);
  TimerEnable(DYNAMICS_TIMER_BASE, DYNAMICS_SUBTIMER);
}

#if USE_UART
//*****************************************************************************
//
// Send a string through UART
//
//*****************************************************************************
void UARTSend(const char *pucBuffer)
{
    unsigned long ulCount = strnlen(pucBuffer, 256);
    
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(PROJ_UART_BASE, *pucBuffer++);
    }
}
#endif

//*****************************************************************************
//
// BMP180 Sensor callback function.  Called at the end of BMP180 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void BMP180AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8BMPDataFlag = 1;
    }
}

//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.    
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }
    // Store the most recent status in case it was an error condition    
	g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a SysTick interrupt, which is used to generate the
// sample interval
//
//*****************************************************************************
void
SysTickIntHandler()
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void HADESI2CIntHandler(void)
{
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Interrupt handler for the timer interrupt associated with the Atmospheric
// sensors.
//
//*****************************************************************************
void AtmosTimerIntHandler(void)
{
  // Clear the interrupt flag
  TimerIntClear(ATMOS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  
  // Make sure data is available, if not, miss this capture and try again
  if(g_vui8BMPDataFlag == 0)
  { 
    // Re-start the data acquisition process
    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
  }
  
  // Reset the data ready flag.
  g_vui8BMPDataFlag = 0;

  // Get a local copy of the latest temperature data in float format.
  BMP180DataTemperatureGetFloat(&g_sBMP180Inst, &fAtmosData.fTemperature);
  
  // Get a local copy of the latest air pressure data in float format.
  BMP180DataPressureGetFloat(&g_sBMP180Inst, &fAtmosData.fPressure);
  
  // Calculate the altitude.
  fAtmosData.fAltitude = 44330.0f * (1.0f - powf(fAtmosData.fPressure / 101325.0f,
                                      1.0f / 5.255f));
  
  // Indicate new data is ready for writing
  newAtmosDataReady = true;
  
  // Re-start the data acquisition process
  BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
}

//*****************************************************************************
//
// Interrupt handler for the timer interrupt associated with the Dynamics
// sensors.
//
//*****************************************************************************
void DynamicsTimerIntHandler(void)
{
  static uint_fast32_t ui32CompDCMStarted = 0;
  static int atmosCount = 0;
  
  // Clear the interrupt flag
  TimerIntClear(DYNAMICS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  
  // Clear the flag
  g_vui8I2CDoneFlag = 0;
  
  // Get floating point version of the Accel Data in m/s^2.
  MPU9150DataAccelGetFloat(&g_sMPU9150Inst, &fDynamicsData.fAccel[XAXIS], 
                           &fDynamicsData.fAccel[YAXIS], &fDynamicsData.fAccel[ZAXIS]);

  // Get floating point version of angular velocities in rad/sec
  MPU9150DataGyroGetFloat(&g_sMPU9150Inst, &fDynamicsData.fGyro[XAXIS],
                          &fDynamicsData.fGyro[YAXIS], &fDynamicsData.fGyro[ZAXIS]);

  // Get floating point version of magnetic fields strength in tesla
  MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, &fDynamicsData.fMag[XAXIS],
                             &fDynamicsData.fMag[YAXIS], &fDynamicsData.fMag[ZAXIS]);
  
  
  // Check if this is our first data ever.
  if(ui32CompDCMStarted == 0)
  {
      // Set flag indicating that DCM is started.
      // Perform the seeding of the DCM with the first data set.
      ui32CompDCMStarted = 1;
      CompDCMMagnetoUpdate(&g_sCompDCMInst, fDynamicsData.fMag[XAXIS],
                             fDynamicsData.fMag[YAXIS], fDynamicsData.fMag[ZAXIS]);
      CompDCMAccelUpdate(&g_sCompDCMInst, fDynamicsData.fGyro[XAXIS],
                          fDynamicsData.fGyro[YAXIS], fDynamicsData.fGyro[ZAXIS]);
      CompDCMGyroUpdate(&g_sCompDCMInst, fDynamicsData.fGyro[XAXIS],
                          fDynamicsData.fGyro[YAXIS], fDynamicsData.fGyro[ZAXIS]);
      CompDCMStart(&g_sCompDCMInst);
      
      // Start the BMP data acquisition process   
      BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
  }
  else
  {
      // DCM Is already started.  Perform the incremental update.
      CompDCMMagnetoUpdate(&g_sCompDCMInst, fDynamicsData.fMag[XAXIS],
                             fDynamicsData.fMag[YAXIS], fDynamicsData.fMag[ZAXIS]);
      CompDCMAccelUpdate(&g_sCompDCMInst, fDynamicsData.fGyro[XAXIS],
                          fDynamicsData.fGyro[YAXIS], fDynamicsData.fGyro[ZAXIS]);
      CompDCMGyroUpdate(&g_sCompDCMInst, -fDynamicsData.fGyro[XAXIS],
                          -fDynamicsData.fGyro[YAXIS], -fDynamicsData.fGyro[ZAXIS]);
      CompDCMUpdate(&g_sCompDCMInst);
  }
  
  // convert mag data to micro-tesla for better human interpretation.
  fDynamicsData.fMag[0] *= 1e6;
  fDynamicsData.fMag[1] *= 1e6;
  fDynamicsData.fMag[2] *= 1e6;
  
  // Indicate data is ready to write
  newDynamicsDataReady = true;
  
  // count towards doing the atmos processing
  atmosCount++;
  if(atmosCount >= ATMOS_SKIP_COUNT)
  {
    AtmosTimerIntHandler();
    atmosCount = 0;
  }
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line, char * msg)
{
  char tempErrStr[128];
    // Set terminal color to red and print error status and locations
    UARTSend("\033[31;1m");
    sprintf(tempErrStr, "Error: %d, File: %s, Line: %d\n\r"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n\r",
               g_vui8ErrorFlag, pcFilename, ui32Line);
    UARTSend(tempErrStr);
    sprintf(tempErrStr, "Msg: %s\n\r", msg);
    UARTSend(tempErrStr);

    // Return terminal color to normal
    UARTSend("\033[0m");

    // Set RGB Color to RED
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[GREEN] = 0x0000;
    g_pui32Colors[BLUE] = 0x0000;
    RGBColorSet(g_pui32Colors);

    // Increase blink rate to get attention
    RGBBlinkRateSet(10.0f);

    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    while(1)
    {
        // Turn on USER LED on
        GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0xFF);
        
        // Delay
        SysCtlDelay(SysCtlClockGet()/30);
        
        // Turn on USER LED off
        GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0x00);
        
        // Delay
        SysCtlDelay(SysCtlClockGet()/30);
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        // Do Nothing
    }

    // If an error occurred call the error handler immediately.
    if(g_vui8ErrorFlag)
    {
      char err_msg[28];
      uint8_t err_num = g_vui8ErrorFlag;
      
      sprintf(err_msg, "Err Flag: %u (0x%x)", err_num , err_num );  
      MPU9150AppErrorHandler(pcFilename, ui32Line, err_msg);
    }

    // clear the data flag for next use.
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    // Clear all the pin interrupts that are set
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        // MPU9150 Data is ready for retrieval and processing.
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}

#if USE_UART
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTSend().
//
//*****************************************************************************
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(PROJ_UART_GPIO_PERIPH);

    // Enable UART
    SysCtlPeripheralEnable(PROJ_UART_PERIPH);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(PROJ_UART_GPIO_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTConfigSetExpClk(PROJ_UART_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}
#endif

//*****************************************************************************
//
// Configure the I2C and its pins.
//
//*****************************************************************************
void ConfigureI2C(void)
{
    // The I2C3 peripheral must be enabled before use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
}

//*****************************************************************************
//
// Configure the timers for collecting data.
//
//*****************************************************************************
void ConfigureTimers(void)
{
  // Turn on the timers used
  //SysCtlPeripheralEnable(ATMOS_TIMER_SYSCTL);
  SysCtlPeripheralEnable(DYNAMICS_TIMER_SYSCTL);
  
  // Configure the timers used
  //TimerConfigure(ATMOS_TIMER_BASE, TIMER_CFG_A_PERIODIC);
  TimerConfigure(DYNAMICS_TIMER_BASE, TIMER_CFG_A_PERIODIC);
  
  // Set the Timer frequencies
  //TimerLoadSet(ATMOS_TIMER_BASE, ATMOS_SUBTIMER, SysCtlClockGet() / ATMOS_FREQ);
  TimerLoadSet(DYNAMICS_TIMER_BASE, DYNAMICS_SUBTIMER, SysCtlClockGet() / DYNAMICS_FREQ);
  
  // Setup Timer Interrupts
  //TimerIntEnable(ATMOS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(DYNAMICS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  //IntEnable(INT_TIMER0A);
  IntEnable(INT_TIMER1A);
  
  // Register the timer ISRs
  //TimerIntRegister(ATMOS_TIMER_BASE, ATMOS_SUBTIMER, AtmosTimerIntHandler);
  TimerIntRegister(DYNAMICS_TIMER_BASE, DYNAMICS_SUBTIMER, DynamicsTimerIntHandler);
  
  // Disable timers until we're ready to use them
  DisableSensorTimers();
}
 
//*****************************************************************************
//
// Configure and Enable the GPIO interrupt. Used for INT signal from the
// MPU9150
//
//*****************************************************************************
void ConfigureGPIObInts(void)
{
    // Power on to GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    
    // Configure Specific Pin
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    
    // Configure Interrupts
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOB);
}

//*****************************************************************************
//
// Configure the LEDs for external indication.
//
//*****************************************************************************
void ConfigureLEDs(void)
{
    //////////LaunchPad 3-clr LED //////////
    // Set the color to a purple approximation.
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[BLUE] = 0x8000;
    g_pui32Colors[GREEN] = 0x0000;

    // Initialize RGB driver.
    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(1.0f);
    RGBEnable();
    
    // Enable blinking indicates config started
    RGBBlinkRateSet(20.0f);
    
    ////////// SENSHUB USER LED //////////
    // Power on to GPIOD for the USER LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    
    // Configure Specific Pin
    GPIOPinTypeGPIOOutput(SENSHUB_LED_PORT, SENSHUB_LED_PIN);
    GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0x00);
}

//*****************************************************************************
//
// Configure what stays on when system goes to sleep
//
//*****************************************************************************
void ConfigureSleep( void )
{
    // Keep only some parts of the systems running while in sleep mode.
    SysCtlPeripheralClockGating(true);
    
    // GPIOB is for the MPU9150 interrupt pin.
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    
    // UART0 is the virtual serial port
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);
    
    // I2C3 is the I2C interface to the ISL29023
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
}

//*****************************************************************************
//
// Configure pushbuttons on senshub booster pack. Purposefully polled.
//
//*****************************************************************************
void ConfigureButtons( void )
{
    // Enable the GPIO port to which the pushbuttons are connected.
    SysCtlPeripheralEnable(BUTTONS_GPIO_PERIPH);

    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(BUTTONS_GPIO_BASE + GPIO_O_CR) |= 0x01;
    HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = 0;

    // Set each of the button GPIO pins as an input with a pull-up.
    GPIODirModeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(BUTTONS_GPIO_BASE, ALL_BUTTONS,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
}

#if USE_SDCARD
//*****************************************************************************
//
// Configure the SD Card, create a data file and write header out to the data 
// file.
//
//*****************************************************************************
void StartSDCard( void)
{
    g_psFlashFile = &dataFile;

    // Initialize the SD card interface
    if( SDCardInit() != OK )
    {
      MPU9150AppErrorHandler(__FILE__, __LINE__, "SD Card Initialization Failure");
    }

    // Write headers to file
    if (f_puts("HADES Data Output\n\n", g_psFlashFile) == EOF) 
                SDCardInit();
    f_sync(g_psFlashFile);
    if (f_puts(",AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Mag X,Mag Y,Mag Z,Press,Temp,Alt\n", g_psFlashFile) == EOF) 
                SDCardInit();
    f_sync(g_psFlashFile);
    if (f_puts(",m/s^2,m/s^2,m/s^2,rad/s,rad/s,rad/s,uT,uT,uT,inHg,C,m\n", g_psFlashFile) == EOF) 
                SDCardInit();
    f_sync(g_psFlashFile);
}
#endif

//*****************************************************************************
//
// convert a float to a two part integers
//
//*****************************************************************************
void floatToDecimals(float inFloat, int_fast32_t *iPart, int_fast32_t *fPart) 
{
    // Conver float value to a integer truncating the decimal part.
    *iPart = (int32_t)inFloat;

    // Multiply by 1000 to preserve first three decimal values.
    // Truncates at the 3rd decimal place.
    *fPart = (int32_t)(inFloat * 1000.0f);

    //
    // Subtract off the integer part from this newly formed decimal
    // part.
    //
    *fPart = *fPart - (*iPart * 1000);

    //
    // make the decimal part a positive number for display.
    //
    if(*fPart < 0)
    {
      *fPart *= -1;
    }
}


//*****************************************************************************
//
// Make data string to write to file.
// Set atmosData to NULL for writing blank instead of atmos data
// Set atmosData AND dynamicsData to NULL to reset the index in the string to 0
//
//*****************************************************************************
int makeDataString(char *outString, dynamicsData_t *dynamicsData, atmosData_t *atmosData)
{
    int len;
    int_fast32_t iIPart[12], iFPart[12]; // Dynamics(9) + Atmos(3)
    static uint32_t idx = 0;
    
    // TODO: check for valid string addr for outStirng
    
    // Reset the idx for printing
    if(dynamicsData == NULL && atmosData == NULL)
    {
      idx = 0;
      return OK;
    }
    
    // Dynamics Data
    floatToDecimals( dynamicsData->fAccel[XAXIS], &iIPart[XAXIS], &iFPart[XAXIS]);
    floatToDecimals( dynamicsData->fAccel[YAXIS], &iIPart[YAXIS], &iFPart[YAXIS]);
    floatToDecimals( dynamicsData->fAccel[ZAXIS], &iIPart[ZAXIS], &iFPart[ZAXIS]);
    floatToDecimals( dynamicsData->fGyro[XAXIS], &iIPart[XAXIS+3], &iFPart[XAXIS+3]);
    floatToDecimals( dynamicsData->fGyro[YAXIS], &iIPart[YAXIS+3], &iFPart[YAXIS+3]);
    floatToDecimals( dynamicsData->fGyro[ZAXIS], &iIPart[ZAXIS+3], &iFPart[ZAXIS+3]);
    floatToDecimals( dynamicsData->fMag[XAXIS], &iIPart[XAXIS+6], &iFPart[XAXIS+6]);
    floatToDecimals( dynamicsData->fMag[YAXIS], &iIPart[YAXIS+6], &iFPart[YAXIS+6]);
    floatToDecimals( dynamicsData->fMag[ZAXIS], &iIPart[ZAXIS+6], &iFPart[ZAXIS+6]);
    
    if(atmosData != NULL)
    {
      // Atmospheric Data
      floatToDecimals( atmosData->fPressure,    &iIPart[9], &iFPart[9] );
      floatToDecimals( atmosData->fTemperature, &iIPart[10], &iFPart[10] );
      floatToDecimals( atmosData->fAltitude,    &iIPart[11], &iFPart[11] );
    
    sprintf(outString,
           "%4u,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d\n\r",
            idx,
            iIPart[0], iFPart[0],         // Accel X
            iIPart[1], iFPart[1],         // Accel Y
            iIPart[2], iFPart[2],         // Accel Z
            iIPart[3], iFPart[3],         // Gyro X
            iIPart[4], iFPart[4],         // Gyro Y
            iIPart[5], iFPart[5],         // Gyro Z
            iIPart[6], iFPart[6],         // Mag X
            iIPart[7], iFPart[7],         // Mag Y
            iIPart[8], iFPart[8],         // Mag Z
            iIPart[7], iFPart[9],         // Pressure
            iIPart[10], iFPart[10],       // Temperature
            iIPart[11], iFPart[11]);      // Altitude
    /*  sprintf(outString,
              "%4u,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n\r",
              idx,
              dynamicsData->fAccel[XAXIS],
              dynamicsData->fAccel[YAXIS],
              dynamicsData->fAccel[ZAXIS],
              dynamicsData->fGyro[XAXIS],
              dynamicsData->fGyro[YAXIS],
              dynamicsData->fGyro[ZAXIS],
              dynamicsData->fMag[XAXIS],
              dynamicsData->fMag[YAXIS],
              dynamicsData->fMag[ZAXIS],
              atmosData->fPressure,
              atmosData->fTemperature,
              atmosData->fAltitude );*/
    }
    else
    {
      sprintf(outString,
           "%4u,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,        ,        ,        \n\r",
            idx,
            iIPart[0], iFPart[0],         // Accel X
            iIPart[1], iFPart[1],         // Accel Y
            iIPart[2], iFPart[2],         // Accel Z
            iIPart[3], iFPart[3],         // Gyro X
            iIPart[4], iFPart[4],         // Gyro Y
            iIPart[5], iFPart[5],         // Gyro Z
            iIPart[6], iFPart[6],         // Mag X
            iIPart[7], iFPart[7],         // Mag Y
            iIPart[8], iFPart[8]);        // Mag Z
    }
    
    idx++;
    
    len = strnlen(outString, MAX_DATASTR_LEN+1);
    if( len > MAX_DATASTR_LEN )
      return ERROR;
    else
      return OK;
}



//*****************************************************************************
//
// Wait for a GO confirmation from the user
// Depending on configuration GO confirmation can be given through the SENSHUB
// pushbuttons or UART.
//
//*****************************************************************************
void waitConfirmGO(void)
{
  bool go = false;
  
  while(!go)
  {
    // Check if any button is pushed
    if((~GPIOPinRead(BUTTONS_GPIO_BASE, ALL_BUTTONS)) & ALL_BUTTONS)
    {
      // wait for delay period
      SysCtlDelay((uint32_t) (GO_DELAY * SysCtlClockGet() / 3) );
      
      // Check if button is still pushed
      if((~GPIOPinRead(BUTTONS_GPIO_BASE, ALL_BUTTONS)) & ALL_BUTTONS)
          go = true;
    }
    
#if USE_UART
    // Check if GO from UART
    if(UARTCharsAvail(PROJ_UART_BASE))
    {
      // Get one char from FIFO
      char inChar = UARTCharGet(PROJ_UART_BASE);
      
      // check if that char means GO
      if(inChar == GO_UART_CHAR)
          go = true;
    }
#endif
  }
}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int main(void)
{
    char dataString[MAX_DATASTR_LEN];
    bool stop = false;
    uint32_t idx = 0;
    
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Disable all interrupts for configuration
    IntMasterDisable();
    
    ////////// Non-Sensor Peripheral Initialization //////////
    
    // Initialize the LEDs for indications
    // Begins blinking the LED at 20 Hz for indicator of configuration time
    ConfigureLEDs();
    
#if USE_UART
    // Initialize the UART.
    ConfigureUART();
    
    // Print the welcome message to the terminal.
    UARTSend("HADES Data Output\n\n\r");
    UARTSend("Configuration Started... ");
#endif
    
#if USE_SDCARD
    StartSDCard();
#endif
    
#ifdef USE_FLASH
    ConfigureFlash();
#endif
    
    // Initialize the I2C
    ConfigureI2C();
    
    // Initialize the Timers
    ConfigureTimers();

    // Initialize GPIO and whatnot for MPU Interrupts
    ConfigureGPIObInts();
    
    // Set which peripherals stay on when system is sleeping
    ConfigureSleep();
    
    // Initialize the buttons for GO confirmation
    ConfigureButtons();
    
    // Delay just to make sure everything is good and set
    SysCtlDelay(SysCtlClockGet() / 3);
    
    // Enable interrupts to the processor.
    // Must be done for sensor configuration since it relies on GPIOb ints
    IntMasterEnable();  

    //////// MPU9150 Initialization ///////
    // Set data ready flag to false
    newDynamicsDataReady = false;
    
    // Initialize I2C3 peripheral.
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             SysCtlClockGet());

    // Initialize the MPU9150 Driver.
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    // Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);

    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    // Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);

    // Configure the data ready interrupt pin output of the MPU9150.
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    // Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);

    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);
       
    ////////// BMP Initialization //////////
    // set data ready flag to false
    newAtmosDataReady = false;
    
    // Initialize the BMP180.
    BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
               BMP180AppCallback, &g_sBMP180Inst);

    // Wait for initialization callback to indicate reset request is complete.
    while(g_vui8BMPDataFlag == 0)
    {    }

    // Reset the flag
    g_vui8BMPDataFlag = 0;
    
    // Enable the system ticks at 10 Hz.
    SysTickPeriodSet(ROM_SysCtlClockGet() / (10 * 3));
    SysTickIntEnable();
    SysTickEnable();
    
    // Stop Blinking to indicate that configuration is complete
    RGBBlinkRateSet(0.0f);  
    // Turn on USERLED to indicate that configuration is complete
    GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0xFF);
    
#if USE_UART
    char tempStr[128];
    
    UARTSend("Done!\n\r");
    sprintf(tempStr, "Enter '%c' or hold button for %fsec to continue to data acquisition\n\r", GO_UART_CHAR, GO_DELAY);
    UARTSend(tempStr);
    UARTSend("... ");
#endif
    
    // Wait for confirmation to continue
    waitConfirmGO();
    
#if USE_UART
    UARTSend("GO confirmed! Begin data acquisition.\n\n\n\r");
    UARTSend("  # , AccelX, AccelY, AccelZ, GyroX , GyroY , GyroZ , Mag X , Mag Y , Mag Z , Press ,  Temp ,   Alt  \n\r");
    UARTSend("    , m/s^2 , m/s^2 , m/s^2 , rad/s , rad/s , rad/s ,   uT  ,   uT  ,   uT  ,  inHg ,   C   ,    m   \n\r");
#endif
    
    // Turn LED Green and Blink for data acquisition
    g_pui32Colors[RED] = 0x0000;
    g_pui32Colors[BLUE] = 0x0000;
    g_pui32Colors[GREEN] = 0x8000;
    RGBColorSet(g_pui32Colors);
    RGBBlinkRateSet(5.0f);
    // Turn off USER LED for data acquisition
    GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0x00);
    
    // Delay for half a second for the human plebians
    SysCtlDelay(SysCtlClockGet() / 3 / 2);
    
    // Start data collection of both sensors
    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
    
    // Turn on Sensor timers to begin data collection
    EnableSensorTimers();
    
    while(!stop)
    {  
        ////////// Get Dynamics Data //////////
        // moved to Dynamics timer ISR
        
        ////////// Get BMP Data //////////
        // moved to ATMOS timer ISR
        
        ////////// Printouts/store data //////////           
        if(newDynamicsDataReady)
        {
          if(newAtmosDataReady) // print both data types
          {
            newDynamicsDataReady = false;
            newAtmosDataReady = false;
            
#if USE_UART || USE_SDCARD
            // make full data string
            if( makeDataString(dataString, &fDynamicsData, &fAtmosData) != OK )
            {
              MPU9150AppErrorHandler(__FILE__, __LINE__, "Data String Creation Failure");
            }
#endif
#if USE_FLASH
            // store data on on-chip flash
            flash_storeDataPoint(&fDynamicsData, &fAtmosData);
#endif
          }
          else  // just print dynamics data
          {
            newDynamicsDataReady = false;
            
#if USE_UART || USE_SDCARD
            // make partial data string
            if( makeDataString(dataString, &fDynamicsData, NULL) != OK )
            {
              MPU9150AppErrorHandler(__FILE__, __LINE__, "Data String Creation Failure");
            }
#endif
#if USE_FLASH
            // store data on on-chip flash
            flash_storeDataPoint(&fDynamicsData, NULL);
#endif
          }
          
#if USE_UART
          UARTSend(dataString);
#endif
            
#if USE_SDCARD
          if (f_puts(dataString, g_psFlashFile) == EOF) 
              SDCardInit();
          f_sync(g_psFlashFile);
#endif
        }
        
        ////////// Watch for stop button press //////////
        // Check if any button is pushed
        if((~GPIOPinRead(BUTTONS_GPIO_BASE, ALL_BUTTONS)) & ALL_BUTTONS)
        {
          // Increment the index for button being pressed
          idx++;
          
          // Check if button is still pushed
          if(idx >= BUTTON_HOLD_COUNT)
            stop = true;
        }
		else
		{
		  // Reset the count because the button was released
		  idx = 0;
		}
#if USE_UART
		// Check for UART stop command
		if(UARTCharsAvail(PROJ_UART_BASE))
		{
		  // Get one char from FIFO
		  char inChar = UARTCharGet(PROJ_UART_BASE);
		  
		  // check if that char means GO
		  if(inChar == STOP_UART_CHAR)
			  stop = true;
		}
#endif

    } // end while(!stop)
	
    DisableSensorTimers();
    
#if USE_UART
    UARTSend("Data collection stopped by command.\n\r");
#endif
    
	// Infinite while loop to end the program
	while(1)
	{
		// Turn LED blue for data acquisition over
		g_pui32Colors[RED] = 0x0000;
		g_pui32Colors[BLUE] = 0x8000;
		g_pui32Colors[GREEN] = 0x0000;
		RGBColorSet(g_pui32Colors);
		RGBBlinkRateSet(0.0f);
		
		// Turn on USER LED for indication data acquisition is done
		GPIOPinWrite(SENSHUB_LED_PORT, SENSHUB_LED_PIN, 0xFF);
		
#if USE_UART
		int confirmStrLen, idx=0;
		
		UARTSend("Enter \""OUTPUT_CONFIRM_STR"\" to send all stored data.\n\r");
		sprintf(tempStr, OUTPUT_CONFIRM_STR);
		
		confirmStrLen = strnlen(tempStr, 20) - 1;
		
		stop = false;	
		while(!stop)
		{
		  // Wait for UART characters
		  while(UARTCharsAvail(PROJ_UART_BASE));
		  
		  // Get one char from FIFO
		  char inChar = UARTCharGet(PROJ_UART_BASE);
		  
		  // check if that char is right
		  if(inChar == tempStr[idx])
		  {
			if(idx >= confirmStrLen)
				// Got through the whole string
				stop = true;
			else
				// move to check the next character next
				idx++;
		  }
		  else
		  {
			// messed up a character, start from the beginning
			idx = 0;
			UARTSend("Character mismatch. Type \""OUTPUT_CONFIRM_STR"\" to output acquired data\n\r");
		  }
		  
		}
		
		UARTSend("Data output confirmed, delaying 2 seconds to read this message.\n\r");
		SysCtlDelay((SysCtlClockGet()/3)*2);
		
		flash_outputData();
		UARTSend("Data Output complete.\n\r");
	}
	
#endif
    
}
