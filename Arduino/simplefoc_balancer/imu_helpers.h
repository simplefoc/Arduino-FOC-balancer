#include "Arduino.h"

#define _PI_2 1.57079632679 
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )


// IMU funcitons declarations
/**
 * Function that checks if the IMU has received new data
 *  - returns 0 if not and 1 if yes
 */
int hasDataIMU();
/*
 * Function initialises the IMU 
 *  - I2C communication
 *  - DMP 
 *  - Configures offsets
 *  - IMU calibration
 *  
 *  returns 0 if error and 1 if well configured
 */
int initIMU();

/**
 * Function reading the pitch value from the IMU
 */
float getPitchIMU();
