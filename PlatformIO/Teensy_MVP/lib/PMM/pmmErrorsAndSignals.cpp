#include <PMM_CONST.h>
#include <cstring>
//--------------Error variables---------------//
#define ERRORS_ARRAY_SIZE 20
#define ERROR_STRING_LENGTH 80
int isThereAnyError = 0, actualNumberOfErrors = 0;
pmmErrorType errorsArray[ERRORS_ARRAY_SIZE];

//----------------Buzzer------------------------//
#if BUZZER_ACTIVATE
/*
    if ((millis() - buzzer_lastTime > 1000 * (1 / BUZZER_OK_FREQ)))
    {
        digitalWrite(BUZZER_PIN, (buzzer_status = !buzzer_status));
        buzzer_lastTime = millis();
    }
*/
#endif
// [1090][763.32s] ERROR 3: S
void
reportError(pmmErrorType errorID, unsigned long timeInMs, unsigned long packetID, int sdIsWorking, int rfIsWorking)
{
    char errorString [ERROR_STRING_LENGTH];
    snprintf(errorString, ERROR_STRING_LENGTH, "[%lu][%.3f] ERROR %i: %s", errorID, timeInMs / 1000.0, errorID, pmmErrorString[errorID]);
    if (actualNumberOfErrors < ERRORS_ARRAY_SIZE)
        errorsArray[actualNumberOfErrors++] = errorID;

}
