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


//*****************************************************************************
//
// Necessary Includes
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/bmp180.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"

#include "drivers/rgb.h"

#include "microSD.h"

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
#define DATA_TYPE               float

#define USE_SDCARD              0    // Set to 1 to use the SD card for storage
#define USE_UART                1    // Set to 1 to use the UART for data output

//*****************************************************************************
//
// Project-Specific Types
//
//*****************************************************************************
typedef struct{
  DATA_TYPE fAccel[3], fGyro[3], fMag[3];
}dynamicsData_t;

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

// Flags
volatile uint_fast8_t g_vui8I2CDoneFlag; // flags to alert main that MPU9150 I2C
                                         // transaction is complete
volatile uint_fast8_t g_vui8ErrorFlag;   // flags to alert main that MPU9150 I2C 
                                         // transaction error has occurred.
volatile uint_fast8_t g_vui8BMPDataFlag; // flags to alert main that BMP180
                                         // data is ready.
volatile uint_fast8_t g_vui8MPUDataFlag; // flags to alert main that MPU9150
                                         // data is ready to be retrieved.

// From microSD.c
extern FIL *g_psFlashFile;

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
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line, char * msg)
{
    // Set terminal color to red and print error status and locations
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);
    UARTprintf("Msg: %s\n", msg);

    // Return terminal color to normal
    UARTprintf("\033[0m");

    // Set RGB Color to RED
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
    RGBColorSet(g_pui32Colors);

    // Increase blink rate to get attention
    RGBBlinkRateSet(10.0f);

    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    while(1)
    {
        // Do Nothing
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

#if USE_UART
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
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
// Make data string to write to file
//
//*****************************************************************************
int makeDataString(char *outString, dynamicsData_t *dynamicsData)
{
  int len;
  int_fast32_t iIPart[9], iFPart[9];
  static uint32_t idx = 0;

  floatToDecimals( dynamicsData->fAccel[XAXIS], &iIPart[XAXIS], &iFPart[XAXIS]);
  floatToDecimals( dynamicsData->fAccel[YAXIS], &iIPart[YAXIS], &iFPart[YAXIS]);
  floatToDecimals( dynamicsData->fAccel[ZAXIS], &iIPart[ZAXIS], &iFPart[ZAXIS]);
  floatToDecimals( dynamicsData->fGyro[XAXIS], &iIPart[XAXIS+3], &iFPart[XAXIS+3]);
  floatToDecimals( dynamicsData->fGyro[YAXIS], &iIPart[YAXIS+3], &iFPart[YAXIS+3]);
  floatToDecimals( dynamicsData->fGyro[ZAXIS], &iIPart[ZAXIS+3], &iFPart[ZAXIS+3]);
  floatToDecimals( dynamicsData->fMag[XAXIS], &iIPart[XAXIS+6], &iFPart[XAXIS+6]);
  floatToDecimals( dynamicsData->fMag[YAXIS], &iIPart[YAXIS+6], &iFPart[YAXIS+6]);
  floatToDecimals( dynamicsData->fMag[ZAXIS], &iIPart[ZAXIS+6], &iFPart[ZAXIS+6]);
  
  
  sprintf(outString,
         "%4u,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d,%3d.%03d\n",
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
  
  idx++;
  
  len = strnlen(outString, MAX_DATASTR_LEN+1);
  if( len > MAX_DATASTR_LEN )
    return ERROR;
  else
    return OK;
}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int main(void)
{
    int_fast32_t i32IPart[16], i32FPart[16];
    uint_fast32_t ui32Idx, ui32CompDCMStarted;
    dynamicsData_t fDynamicsData;
    float pfData[8];
    char dataString[MAX_DATASTR_LEN];

    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Enable port B used for motion interrupt.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

#if USE_UART
    // Initialize the UART.
    ConfigureUART();
    
    // Print the welcome message to the terminal.
    UARTprintf("\033[1;1HHADES Data Output\n\n");
    UARTprintf("\033[3;1H      AccelX  AccelY  AccelZ   GyroX   GyroY   GyroZ   Mag X   Mag Y   Mag Z",dataString);
#endif
    
#if USE_SDCARD
    if( SDCardInit() != OK )
    {
      MPU9150AppErrorHandler(__FILE__, __LINE__, "SD Card Initialization Failure");
    }
#endif
    
    // Set the color to a purple approximation.
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[BLUE] = 0x8000;
    g_pui32Colors[GREEN] = 0x0000;

    // Initialize RGB driver.
    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(1.0f);
    RGBEnable();

    // Initialize the I2C
    ConfigureI2C();

    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOB);

    // Keep only some parts of the systems running while in sleep mode.
    // GPIOB is for the MPU9150 interrupt pin.
    // UART0 is the virtual serial port
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    // I2C3 is the I2C interface to the ISL29023
    SysCtlPeripheralClockGating(true);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

    // Enable interrupts to the processor.
    IntMasterEnable();

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
    
    // Enable blinking indicates config finished successfully
    RGBBlinkRateSet(1.0f);

    ui32CompDCMStarted = 0;

    while(1)
    {
        // Go to sleep mode while waiting for data ready.
        while(!g_vui8I2CDoneFlag)
        {
            SysCtlSleep();
        }

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

        // Increment the skip counter.  Skip counter is used so we do not
        // overflow the UART with data.
        g_ui32PrintSkipCounter++;
        if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
        {
            // Reset skip counter.
            g_ui32PrintSkipCounter = 0;

            // convert mag data to micro-tesla for better human interpretation.
            fDynamicsData.fMag[0] *= 1e6;
            fDynamicsData.fMag[1] *= 1e6;
            fDynamicsData.fMag[2] *= 1e6;

            // convert the dynamics data to integers
            for(ui32Idx = XAXIS; ui32Idx <= ZAXIS; ui32Idx++)
            {
                floatToDecimals(fDynamicsData.fAccel[ui32Idx], &i32IPart[ui32Idx], &i32FPart[ui32Idx]);
                floatToDecimals(fDynamicsData.fGyro[ui32Idx], &i32IPart[ui32Idx+3], &i32FPart[ui32Idx+3]);
                floatToDecimals(fDynamicsData.fMag[ui32Idx], &i32IPart[ui32Idx+6], &i32FPart[ui32Idx+6]);
            }
            
            // decompose the floats of pfData into a integer part and a
            // fraction (decimal) part.
            for(ui32Idx = 0; ui32Idx < 7; ui32Idx++)
            {
                floatToDecimals(pfData[ui32Idx], &i32IPart[ui32Idx+9], &i32FPart[ui32Idx+9]);
            }
            
            if( makeDataString(dataString, &fDynamicsData) != OK )
            {
              MPU9150AppErrorHandler(__FILE__, __LINE__, "Data String Creation Failure");
            }      
            
#if USE_UART
            UARTprintf("\033[4;1H%s",dataString);
#endif
            
#if USE_SDCARD
            if (f_puts(dataString, g_psFlashFile) == EOF) 
                SDCardInit();
            f_sync(g_psFlashFile);
#endif

        }
    }
}
