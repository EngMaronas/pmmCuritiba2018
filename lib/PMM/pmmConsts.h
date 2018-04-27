#ifndef PMM_CONST_h
#define PMM_CONST_h

#define DEBUG_SERIAL 1



#if DEBUG_SERIAL
    #define DEBUG_PRINT(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x) do {} while (0)
#endif

#define DEBUG_MAINLOOP_SERIAL 0

#if DEBUG_MAINLOOP_SERIAL
    #define DEBUG_MAINLOOP_PRINT(x) Serial.println(x)
#else
    #define DEBUG_MAINLOOP_PRINT(x) do {} while (0)
#endif

//-------------- LoRa ----------------//
#define PIN_RFM95_CS 15
#define PIN_RFM95_RST 17
#define PIN_RFM95_INT 16
#define RF95_FREQ 915.0
#define RF_WORDS_IN_PACKET 24
#define RF_BYTES_IN_PACKET (RF_WORDS_IN_PACKET * 4)
#define RF_INIT_MAX_TRIES 20
const char RF_VALIDATION_HEADER[4] = {'V', 'R', 'N', 'M'}; // Make sure that there is no NULL-terminator char. This one is inverted, as it will sent inverted (little-endian).
const char RF_VALIDATION_HEADER_EXTRA[5] = {"MNEX"};

//--------------- LEDS ---------------//
#define PIN_LED_ERRORS 3 // Red
#define PIN_LED_RECOVERY 1 // Blue
#define PIN_LED_ALL_OK_AND_RF 4 // Green

//-------------- Buzzer --------------//
#define BUZZER_ACTIVATED 0
#define BUZZER_PIN  2

// -------------- GPS ----------------//
#define GPS_SENTENCE_SIZE 80

// --------------- SD ----------------//
#define FILENAME_MAX_LENGTH 20
#define LOG_BUFFER_LENGTH 1000
//#define SD_CHIP_SELECT BUILTIN_SDCARD // Change if different SD card

// Below unused, remove when all ok.
// Max of 12 chars (13 with \0, SD.h nonsenses)
// 123456789012|
// PMM_000e.csv

//-------------- DELAY ---------------//
#define DELAY_MS_RECUPERATION_CHECK 100
#define DELAY_MS_BAROMETER 50
#define DELAY_MS_RF 180

const char FILENAME_BASE_PREFIX[] = {"PMM_"};
const char FILENAME_BASE_SUFFIX[] = {".csv"};
const char FILENAME_EXTRA_SUFFIX[] = {"e.txt"};

#endif
