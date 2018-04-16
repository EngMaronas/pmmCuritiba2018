#ifndef PMM_CONST_h
#define PMM_CONST_h

//-------------- LoRa ----------------//
#define PIN_RFM95_CS 15
#define PIN_RFM95_RST 17
#define PIN_RFM95_INT 16
#define RF95_FREQ 915.0
#define RF_WORDS_IN_PACKET 17
#define RF_BYTES_IN_PACKET (RF_WORDS_IN_PACKET * 4)
#define RF_INIT_MAX_TRIES 20
const char RF_VALIDATION_HEADER[4] = {'V', 'R', 'N', 'M'}; // Make sure that there is no NULL-terminator char.

//--------------- LEDS ---------------//
#define PIN_LED_AVIONIC 3 // Red
#define PIN_LED_RECOVERY 1 // Blue
#define PIN_LED_ALL_OK_AND_RF 4 // Green

//-------------- Buzzer --------------//
#define BUZZER_ACTIVATE 1
#define BUZZER_PIN  2

typedef enum {
    OK,
    ERROR_SD,
    ERROR_RF_INIT,
    ERROR_RF_SET_FREQ,
    ERROR_ACCELEROMETER_INIT,
    ERROR_GYROSCOPE_INIT,
    ERROR_MAGNETOMETER_INIT,
    ERROR_BAROMETER_INIT,
    ERRORS_AMOUNT
} pmmErrorType;






#endif
