#ifndef PMM_ERRORS_AND_SIGNALS_h
#define PMM_ERRORS_AND_SIGNALS_h

#include <RH_RF95.h>

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

class PmmErrorsAndSignals
{
private:
    int actualNumberOfErrors = 0;
    int systemWasOk = 1, signalIsOn = 0, signalStarterCounter = 0, signalActualErrorIndex = 0, signalActualErrorCounter = 0;
    unsigned long millisNextSignalState = 0;
    char errorString[ERROR_STRING_LENGTH], filenameExtra[FILENAME_MAX_LENGTH];
    RH_RF95 *rf95Ptr;
    pmmErrorType errorsArray[ERRORS_ARRAY_SIZE];
    File mFileExtra;
    const char* returnPmmErrorString(pmmErrorType errorId);
    void writeToSd(char *stringToWrite);

public:
    PmmErrorsAndSignals(RH_RF95 *rf95Ptr, uint16_t fileID);
    void updateLedsAndBuzzer();
    void reportError(pmmErrorType errorID, unsigned long timeInMs, unsigned long packetID, int sdIsWorking, int rfIsWorking);
};


#endif
