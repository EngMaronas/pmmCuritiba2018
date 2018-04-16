#include <pmmConsts.h>

//-------------- GPS Venus Function ---------------//

void gps_getField(char* originalGpsSentence, char* buffer, int index)
{
    int sentencePos = 0, fieldPos = 0, commaCount = 0;
    while (sentencePos++ < GPS_SENTENCE_SIZE)
    {
        if (originalGpsSentence[sentencePos] == ',')
        {
            commaCount ++;
            sentencePos ++;
        }
        if (commaCount == index)
            buffer[fieldPos++] = originalGpsSentence[sentencePos];
    }
    buffer[fieldPos] = '\0';
}
