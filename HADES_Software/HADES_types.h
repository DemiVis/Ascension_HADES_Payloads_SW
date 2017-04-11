//*****************************************************************************
//
// HADES_types.h - types used throughout the HADES payload software project.
//
// This software is to be used on the Payloads of the HADES project by Team
// Ascension. The types contained are used throughout the HADES software.
//
// Primary Author: Matthew Demi Vis
//                 vism@my.erau.edu
// Others Responsible: Jason Lathbury        Garrett Trahern
//                    lathburj1@my.erau.edu  traherng@my.erau.edu
//
//*****************************************************************************

#ifndef HADES_TYPES
#define HADES_TYPES

#define DATA_TYPE               float

typedef struct{
    DATA_TYPE fAccel[3], fGyro[3], fMag[3];
}dynamicsData_t;
#define DYNAMICS_STRUCT_SZ      (sizeof(DATA_TYPE)*9)

typedef struct{
    DATA_TYPE fTemperature, fPressure, fAltitude;
}atmosData_t;
#define ATMOS_STRUCT_SZ         (sizeof(DATA_TYPE)*3)

typedef struct{
	dynamicsData_t dynamics_data;
	atmosData_t atmos_Data;
}sensorData_t;
#define SENSOR_STRUCT_SZ        (DYNAMICS_STRUCT_SZ + ATMOS_STRUCT_SZ)

#endif