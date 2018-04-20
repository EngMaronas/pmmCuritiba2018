#include <pmmConsts.h>
#include <cstring>
#include <RH_RF95.h>
#include <SD.h>
#include <pmmErrorsAndSignals.h>

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
    ERROR_SD_WRITE,
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
    "SD write fail",            // ERROR_SD_WRITE,
    "RF init fail",             // ERROR_RF_INIT,
    "RF Set Freq fail",         // ERROR_RF_SET_FREQ,
    "Accelerometer init fail",  // ERROR_ACCELEROMETER_INIT,
    "Gyroscope init fail",      // ERROR_GYROSCOPE_INIT,
    "Magnetometer init fail",   // ERROR_MAGNETOMETER_INIT,
    "Barometer init fail",      // ERROR_BAROMETER_INIT
    "Programming error"         // ERROR_PROGRAMMING
};
const char *recuperationActivatedString = "Recuperation Activated!";


PmmErrorsAndSignals::PmmErrorsAndSignals()
{
}

// Initializer
void PmmErrorsAndSignals::init(RH_RF95 *rf95Ptr, uint16_t fileID)
{
    // Init variables values
    mRf95Ptr = rf95Ptr;
    mActualNumberOfErrors = 0;
    snprintf(mFilenameExtra, FILENAME_MAX_LENGTH, "%s%03u%s%s", FILENAME_BASE_PREFIX, fileID, FILENAME_EXTRA_SUFFIX, FILENAME_EXTRA_EXTENSION); // Declaration of the filename of the extra log

    mSystemWasOk = mIsShortBeepOfSystemWasOk = 1; mSignalIsOn = mSignalStarterCounter = mSignalActualErrorIndex = mSignalActualErrorCounter = 0;
    mMillisNextSignalState = 0;

    pinMode(PIN_LED_RECOVERY, OUTPUT);
    pinMode(PIN_LED_ERRORS, OUTPUT);
    pinMode(PIN_LED_ALL_OK_AND_RF, OUTPUT);
    pinMode(BUZZER_PIN,OUTPUT);

    digitalWrite(PIN_LED_RECOVERY, LOW);
    digitalWrite(PIN_LED_ERRORS, LOW);
    digitalWrite(PIN_LED_ALL_OK_AND_RF, HIGH); // Initializes it HIGH.
    digitalWrite(BUZZER_PIN, LOW);
}

const char* PmmErrorsAndSignals::returnPmmErrorString(pmmErrorType errorId)
{
    if (errorId < 0 or errorId >= ERRORS_AMOUNT)
        return pmmErrorString[ERROR_PROGRAMMING];
    else
        return pmmErrorString[errorId];
}

void PmmErrorsAndSignals::writelnToSd(char *stringToWrite, char *filename)
{
    File fileExtra;
    fileExtra = SD.open(filename, FILE_WRITE);
    if (fileExtra)
    {
        fileExtra.println(stringToWrite);
        fileExtra.close();
    }
}




//       ..........     ..........
//  .....          .....          .....
//
void PmmErrorsAndSignals::updateLedsAndBuzzer()
{
    if (millis() >= mMillisNextSignalState)
    {
        if (mSystemWasOk)
        {
            if (!mSignalIsOn) // If signal not On
            {
                mSignalIsOn = 1;
                #if BUZZER_ACTIVATED
                    digitalWrite(BUZZER_PIN, HIGH); // Turn On
                #endif
                if (mIsShortBeepOfSystemWasOk)
                {
                    mMillisNextSignalState = millis() + 100; // Long beep On
                }
                else
                {
                    mMillisNextSignalState = millis() + 1000; // Long beep On
                }
            }
            else // So signal is On
            {
                mSignalIsOn = 0;
                #if BUZZER_ACTIVATED
                    digitalWrite(BUZZER_PIN, LOW); // Turn Off
                #endif
                if (mIsShortBeepOfSystemWasOk)
                {
                    mMillisNextSignalState = millis() + 100; // Short beep Off
                    mIsShortBeepOfSystemWasOk = 0;
                }
                else
                {
                    mIsShortBeepOfSystemWasOk = 1;
                    mMillisNextSignalState = millis() + 1000; // Long beep Off
                    if (mActualNumberOfErrors)
                        mSystemWasOk = 0; // So the signal will have a low state of >500ms before the error signal
                }
            }

        }
        else // System has an error
        {
            if (!mSignalIsOn) // If signal not On
            {
                mSignalIsOn = 1;
                #if BUZZER_ACTIVATED
                    digitalWrite(BUZZER_PIN, HIGH); // Turn On
                #endif
                digitalWrite(PIN_LED_ERRORS, HIGH);

                if (!mSignalActualErrorCounter) // If is a header beep
                {
                    if (!mSignalStarterCounter) // mSignalStarterCounter needed to don't keep assigning the value
                    {
                        if (mSignalActualErrorIndex == 0)
                        {
                            mSignalStarterCounter = 3; // The first error has 3 short signals before the error code
                        }
                        else
                        {
                            mSignalStarterCounter = 2; // The subsequent errors has 2 short beeps before the error code
                        }
                    }
                    mMillisNextSignalState = millis() + 100; // Short Beep High - Make small signals to show the start of an error.
                }
                else // Is an error code beep
                    mMillisNextSignalState = millis() + 300; // Medium Beep Low
            }
            else // So signal is On
            {
                mSignalIsOn = 0;
                #if BUZZER_ACTIVATED
                    digitalWrite(BUZZER_PIN, LOW); // Turn Off
                #endif
                digitalWrite(PIN_LED_ERRORS, LOW);
                if (mSignalStarterCounter) // Is a header beep
                {
                    if (--mSignalStarterCounter > 0)
                    {
                        mMillisNextSignalState = millis() + 100; // Short Beep Low
                    }
                    else // If the signal starter is now 0, start the error code signal.
                    {
                        mSignalActualErrorCounter = mErrorsArray[mSignalActualErrorIndex]; // Short beeps are over, load the next error
                        mMillisNextSignalState = millis() + 500;
                    }
                }
                else //Is an error beep
                {
                    if (--mSignalActualErrorCounter > 0) // If is a low level between errors beeps
                        mMillisNextSignalState = millis() + 500; // Medium Beep Low
                    else // Is a low lever after all the errors beeps, get the next error ID (go to the first one again)
                    {
                        mMillisNextSignalState = millis() + 1000; // Medium Beep Low
                        if ((++mSignalActualErrorIndex) >= mActualNumberOfErrors) // If the index++ is bigger than the maximum index value
                            mSignalActualErrorIndex = 0;
                    }
                }
            }
        }
    }
}

// [1090][763.32s] ERROR 3: SD
void PmmErrorsAndSignals::reportError(pmmErrorType errorID, unsigned long packetID, int sdIsWorking, int rfIsWorking)
{
    char errorString[ERROR_STRING_LENGTH];

    digitalWrite(PIN_LED_ALL_OK_AND_RF, LOW); // Make sure the All OK Led is Off (or turn it off if first time)
    snprintf(errorString, ERROR_STRING_LENGTH, "[%lu][%.3f] ERROR %i: %s", packetID, millis() / 1000.0, errorID, returnPmmErrorString(errorID));
    Serial.println(errorString); //Initialize Serial Port at 9600 baudrate. // debug
    if (mActualNumberOfErrors < ERRORS_ARRAY_SIZE)
        mErrorsArray[mActualNumberOfErrors++] = errorID;
    if (sdIsWorking)
        writelnToSd(errorString, mFilenameExtra);
    if (rfIsWorking)
        mRf95Ptr->send((uint8_t*)errorString, strlen(errorString) + 1);
}

// [1090][763.32s] ERROR 3: SD
void PmmErrorsAndSignals::reportRecuperation(unsigned long packetID, int sdIsWorking, int rfIsWorking)
{
    char recuperationString[ERROR_STRING_LENGTH];
    snprintf(recuperationString, ERROR_STRING_LENGTH, "%s[%lu][%.3f] %s", RF_VALIDATION_HEADER_EXTRA, packetID, millis() / 1000.0, recuperationActivatedString);
    digitalWrite(PIN_LED_RECOVERY, HIGH);
    if (sdIsWorking)
        writelnToSd(recuperationString + 4, mFilenameExtra); // +4 to skip the MNEX header
    if (rfIsWorking)
        mRf95Ptr->send((uint8_t*)recuperationString, strlen(recuperationString) + 1);

}
void PmmErrorsAndSignals::blinkRfLED(int state)
{
    if (!mActualNumberOfErrors)
        digitalWrite(PIN_LED_ALL_OK_AND_RF, state);
}
