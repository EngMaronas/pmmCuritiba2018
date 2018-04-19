#ifndef PMM_CONST_h
#define PMM_CONST_h

#define DEBUG_SERIAL 1

//-------------- LoRa ----------------//
#define PIN_RFM95_CS 15
#define PIN_RFM95_RST 17
#define PIN_RFM95_INT 16
#define RF95_FREQ 915.0
#define RF_WORDS_IN_PACKET 17
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

// Max of 12 chars (13 with \0, SD.h nonsenses)
// 123456789012|
// PMM_000e.csv
const char FILENAME_BASE_PREFIX[] = "PMM_";
const char FILENAME_EXTRA_SUFFIX[] = "e";
const char FILENAME_BASE_EXTENSION[] = ".csv";
const char FILENAME_EXTRA_EXTENSION[] = ".txt";


#endif
