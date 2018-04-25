#ifndef PMM_GENERAL_FUNCTIONS_h
#define PMM_GENERAL_FUNCTIONS_h

#include <pmmConsts.h>
#include <SdFat.h>

void gps_getField(char* originalGpsSentence, char* buffer, int index);

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
    int writeToFile(char *arrayToWrite, int32_t length);
    bool sdBusy();
    void getFilename(char *stringToReturn, uint32_t bufferLength);
};

#endif
