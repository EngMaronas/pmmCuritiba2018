#include <pmmConsts.h>
#include <cstring>
#include <RH_RF95.h>
#include <SD.h>
//--------------Error variables---------------//
#define ERRORS_ARRAY_SIZE 20
#define ERROR_STRING_LENGTH 80

//----------------Buzzer------------------------//
#if BUZZER_ACTIVATED
/*
    if ((millis() - buzzer_lastTime > 1000 * (1 / BUZZER_OK_FREQ)))
    {
        digitalWrite(BUZZER_PIN, (buzzer_status = !buzzer_status));
        buzzer_lastTime = millis();
    }
*/
#endif

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
    ERROR_PROGRAMMING,
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
    "Barometer init fail",      // ERROR_BAROMETER_INIT
    "Programming error"         // ERROR_PROGRAMMING
};
const char *recuperationActivatedString = "Recuperation Activated!";





// Buzzer is 1s/0.5s if all ok.
// RED LED = off, is = buzzer if errors found.

class PmmErrorsAndSignals
{
private:
    int actualNumberOfErrors = 0;
    pmmErrorType errorsArray[ERRORS_ARRAY_SIZE];
    char errorString[ERROR_STRING_LENGTH], filenameExtra[FILENAME_MAX_LENGTH];
    RH_RF95 *rf95Ptr;
    File fileExtra;

    const char* returnPmmErrorString(pmmErrorType errorId)
    {
        if (errorId < 0 or errorId >= ERRORS_AMOUNT)
            return pmmErrorString[ERROR_PROGRAMMING];
        else
            return pmmErrorString[errorId];
    }

    void writeToSd(char *stringToWrite)
    {
        fileExtra = SD.open(filenameExtra, FILE_WRITE);
        if (fileExtra)
        {
            fileExtra.println(stringToWrite);
            fileExtra.close();
        }
    }

    void
public:
    PmmErrorsAndSignals(RH_RF95 *rf95Ptr, uint8_t fileID)
    {
        snprintf(filenameExtra, FILENAME_MAX_LENGTH, "%s%u%s%s", FILENAME_BASE_PREFIX, fileID, FILENAME_EXTRA_SUFFIX, FILENAME_EXTENSION);
        pinMode(PIN_LED_RECOVERY, OUTPUT);
        pinMode(PIN_LED_ERRORS, OUTPUT);
        pinMode(PIN_LED_ALL_OK_AND_RF, OUTPUT);
        pinMode(BUZZER_PIN,OUTPUT);

        digitalWrite(PIN_LED_RECOVERY, LOW);
        digitalWrite(PIN_LED_ERRORS, LOW);
        digitalWrite(PIN_LED_ALL_OK_AND_RF, LOW);
        digitalWrite(BUZZER_PIN, LOW);
    }
    void updateLedsAndBuzzer()
    {

    }
    // [1090][763.32s] ERROR 3: SD

    void reportError(pmmErrorType errorID, unsigned long timeInMs, unsigned long packetID, int sdIsWorking, int rfIsWorking)
    {

        snprintf(errorString, ERROR_STRING_LENGTH, "[%lu][%.3f] ERROR %i: %s", packetID, timeInMs / 1000.0, errorID, returnPmmErrorString(errorID));
        if (actualNumberOfErrors < ERRORS_ARRAY_SIZE)
        {
            errorsArray[actualNumberOfErrors++] = errorID;
        }
    }
};
