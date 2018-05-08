#ifndef PMM_SD_h
#define PMM_SD_h

#include <Arduino.h>
#include <pmmConsts.h>
#include <SdFat.h>

class SdManager
{
private:
    // Private functions
    void yield();
    SdFatSdioEX mSdEx;
    File mFile;
    char mFilename[FILENAME_MAX_LENGTH];

public:
    SdManager();
    int init();
    void setFilename(char *filename);
    int setFilenameAutoId(const char* baseName, const char* suffix);
    int writeToFilename(char *filename, char *arrayToWrite, int32_t length);
    int writeStringToFilename(char *filename, char *arrayToWrite);
    int writeToFile(char *arrayToWrite, int32_t length);
    int writeToFile(char *arrayToWrite);
    bool sdBusy();
    void getFilename(char *stringToReturn, uint32_t bufferLength);
};

#endif
