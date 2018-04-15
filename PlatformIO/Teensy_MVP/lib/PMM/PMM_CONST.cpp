#include <PMM_CONST.h>

/*
typedef enum {
    OK,
    ERROR_SD,
    ERROR_RF_INIT,
    ERROR_RF_SET_FREQ,
    ERROR_ACCELEROMETER_INIT,
    ERROR_GYROSCOPE_INIT,
    ERROR_MAGNETOMETER_INIT,
    ERROR_BAROMETER_INIT,
    ERRORS_AMOUNT
} pmmErrorType;
*/

const char *pmmErrorString[ERRORS_AMOUNT] = {
    "No errors",                // OK,
    "SD init fail",             // ERROR_SD,
    "RF init fail",             // ERROR_RF_INIT,
    "RF Set Freq fail",         // ERROR_RF_SET_FREQ,
    "Accelerometer init fail",  // ERROR_ACCELEROMETER_INIT,
    "Gyroscope init fail",      // ERROR_GYROSCOPE_INIT,
    "Magnetometer init fail",   // ERROR_MAGNETOMETER_INIT,
    "Barometer init fail"       // ERROR_BAROMETER_INIT
};

const char *recuperationActivatedString = "Recuperation Activated!";
