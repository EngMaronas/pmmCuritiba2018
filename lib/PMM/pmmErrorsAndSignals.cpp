#include <pmmConsts.h>
#include <cstring>
#include <RH_RF95.h>
#include <SD.h>
#include <PmmErrorsAndSignals.h>

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

const char* PmmErrorsAndSignals::returnPmmErrorString(pmmErrorType errorId)
{
    if (errorId < 0 or errorId >= ERRORS_AMOUNT)
        return pmmErrorString[ERROR_PROGRAMMING];
    else
        return pmmErrorString[errorId];
}

void PmmErrorsAndSignals::writeToSd(char *stringToWrite)
{
    mFileExtra = SD.open(filenameExtra, FILE_WRITE);
    if (fileExtra)
    {
        fileExtra.println(stringToWrite);
        fileExtra.close();
    }
}

PmmErrorsAndSignals::PmmErrorsAndSignals(RH_RF95 *rf95Ptr, uint16_t fileID)
{
    this->rf95Ptr = rf95Ptr;
    actualNumberOfErrors = 0;
    systemWasOk = 1;
    signalIsOn = signalStarterCounter = signalActualErrorIndex = signalActualErrorCounter = 0;
    millisNextSignalState = 0;


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

void PmmErrorsAndSignals::updateLedsAndBuzzer()
{
    if (millis() >= millisNextSignalState)
    {
        if (systemWasOk)
        {
            if (signalIsOn)
            {
                digitalWrite(BUZZER_PIN, LOW); // Turn Off
                millisNextSignalState = millis() + 500;
                if (actualNumberOfErrors)
                    systemWasOk = 0; // So the signal will have a low state of >500ms before the error signal
            }
            else // Signal is Off
            {
                digitalWrite(BUZZER_PIN, HIGH); // Turn On
                millisNextSignalState = millis() + 1000;
            }


        else // System has an error
            if (signalIsOn)
            {
                digitalWrite(BUZZER_PIN, LOW); // Turn Off

                if (signalStarterCounter)
                {
                    signalStarterCounter--;
                    millisNextSignalState = millis() + 10;
                }
                if (!signalStarterCounter) // If the signal starter is 0, start the error code signal.
                    millisNextSignalState = millis() + 100;
                    signalActualErrorCounter = errorsArray[signalActualErrorIndex]
            }
            else // Signal is Off
            {
                digitalWrite(BUZZER_PIN, HIGH); // Turn On
                if (signalActualErrorCounter)
                {
                    millisNextSignalState = millis() + 100;
                    signalActualErrorCounter --;
                    if
                }
                else
                {
                    if (signalActualErrorIndex == 0 and signalStarterCounter == 0) // signalStarterCounter needed to don't keep assigning the value
                        signalStarterCounter = 3; // The first error has 3 short signals before the error code
                    else if (signalActualErrorIndex > 0 and signalStarterCounter == 0)
                        signalStarterCounter = 2; // The subsequent errors has 2 short beeps before the error code
                    if (signalStarterCounter)
                        millisNextSignalState = millis() + 10; // Make small signals to show the start of an error.
                }
            }
        }
    }
}

// [1090][763.32s] ERROR 3: SD
void PmmErrorsAndSignals::reportError(pmmErrorType errorID, unsigned long timeInMs, unsigned long packetID, int sdIsWorking, int rfIsWorking)
{
    snprintf(errorString, ERROR_STRING_LENGTH, "[%lu][%.3f] ERROR %i: %s", packetID, timeInMs / 1000.0, errorID, returnPmmErrorString(errorID));
    if (actualNumberOfErrors < ERRORS_ARRAY_SIZE)
        errorsArray[actualNumberOfErrors++] = errorID;

}
