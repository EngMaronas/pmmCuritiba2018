#ifndef PMM_ERRORS_AND_SIGNALS_h
#define PMM_ERRORS_AND_SIGNALS_h

#include <RH_RF95.h>
#include <pmmConsts.h>
#include <pmmSd.h>

//--------------Error variables---------------//
#define ERRORS_ARRAY_SIZE 20
#define ERROR_STRING_LENGTH 80

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

class PmmErrorsAndSignals
{
private:
    SdManager mSdManager;
    RH_RF95 *mRf95Ptr; // Pointer to the RF object
    pmmErrorType mErrorsArray[ERRORS_ARRAY_SIZE]; // Total erros in the system
    int mActualNumberOfErrors; // Total errors in the system number
    char mFilenameExtra[FILENAME_MAX_LENGTH]; // Filename of this extra log

    int mSystemWasOk, mSignalIsOn, mSignalStarterCounter, mSignalActualErrorIndex, mSignalActualErrorCounter, mIsShortBeepOfSystemWasOk; // Used updateLedsAndBuzzer
    unsigned long mMillisNextSignalState; // Used updateLedsAndBuzzer


public:
    PmmErrorsAndSignals();
    void init(RH_RF95 *rf95Ptr, uint16_t fileID);
    void updateLedsAndBuzzer();
    void reportError(pmmErrorType errorID, unsigned long packetID, int sdIsWorking, int rfIsWorking);
    void reportRecuperation(unsigned long packetID, int sdIsWorking, int rfIsWorking);
    void blinkRfLED(int state);
};

#endif
