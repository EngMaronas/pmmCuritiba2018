/*  Code written by Marcelo Maroñas, Eduardo Alves e Lucas Ribeiro @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - March 06, 2018
 *  This is an adaptation necessary for using the GY80 IMU module with the microcontroller Teensy version 3.6.
 *  This the very basic code using the library.The library was written using the best libraries I could find at the moment for each GY80 sub-module (Gyro, Accel, Magne, BMP085) and
 *  putting them together in an lightweight and easy to understand code.Dont use it with Arduino, there's a lighter version of GY80 library that doesnt need so much memory, check in my GitHub.
 *  The libraries for each sub-modules are, in majority, adapted adafruit libraries, and because of it,
 *  they are very heav.But in the counterpart, they also are very robust and have methods for everything that you need to do with the sensor.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[3]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[3]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[3]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[3]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas

 * Conexões
 * IMU:
 * Teensy 3.6 (3.3V) ----------------> VDD (Pin 3)
 * GND ------------------------------> GND
 * SCL0 (Pin 19) --------------------> SCL
 * SDA0 (Pin 20) --------------------> SDA
 *
 * Lora:
 * Teensy 3.6 (3.3V) ----------------> Vin
 * GND ------------------------------> GND
 * Pin 16 ---------------------------> G0
 * SCK0(Pin 13) ---------------------> SCK
 * MISO0 (Pin 12) -------------------> MISO
 * MOSI0 (Pin 11) -------------------> MOSI
 * Pin 15 ---------------------------> CS
 * pin 17 ---------------------------> RTS
 *
 * GPS:
 * Teensy 3.6 (3.3V) ----------------> Vcc in
 * GND-------------------------------> GND
 * RX4 (Pin 31) ---------------------> TX
 * TX4 (Pin 32) ---------------------> RX
 *
 *  * Caso queiria usar outra entrada RX/TX consultar pinagem em: https://www.pjrc.com/teensy/pinout.html
 * E trocar Serial2 pelo numero do Serial a utilizar
 */

//---------------Inclusão de bibliotecas---------------//
#include <GY80TEENSY.h>
#include <RH_RF95.h>
#include <SD.h>
#include <SPI.h>
#include <PMM_CONST.h>

//---------------Definições e variáveis Lora-------------------//
#define PIN_RFM95_CS 15
#define PIN_RFM95_RST 17
#define PIN_RFM95_INT 16
#define RF95_FREQ 915.0
#define RF_WORDS_IN_PACKET 17
#define RF_BYTES_IN_PACKET (RF_WORDS_IN_PACKET * 4)
#define RF_INIT_MAX_TRIES 10

const char RF_VALIDATION_HEADER[4] = {'V', 'R', 'N', 'M'}; // Make sure that there is no NULL-terminator char.
float rf_packetTime, rf_packetID;

RH_RF95 rf95(PIN_RFM95_CS, PIN_RFM95_INT);

//---------------Variáveis GPS Venus---------------//
#define GPS_SENTENCE_SIZE 80
char gps_sentence[GPS_SENTENCE_SIZE];
float gps_lat, gps_lon;
char gps_stringBuffer[20];

//-------------- Definições Buzzer ----------------//
#define BUZZER_ACTIVATE 1
#define BUZZER_PIN  2
#define BUZZER_OK_FREQ 100
unsigned long buzzer_lastTime = 0;
bool buzzer_status = false;

//---------------Variáveis módulo SD---------------//
File fileLog;
const int chipSelect = BUILTIN_SDCARD;

//--------------Declaração do Struct---------------//
IMU_s imu_struct;
IMU_s *imu_pstruct = &imu_struct;

//--------------Error variables---------------//
#define ERRORS_ARRAY_SIZE 20
int sdIsWorking = 1; //Prints the values of IMU_s in SD card
int rfIsWorking = 1;
int isThereAnyError = 0;
uint8_t errorsArray[ERRORS_ARRAY_SIZE];

//---------------Definições dos LEDS---------------//
#define PIN_LED_AVIONIC 3 // Red
#define PIN_LED_RECOVERY 1 // Blue
#define PIN_LED_ALL_OK_AND_RF 4 // Green
#define LED_AVIONIC_INTERVAL 100 // Will keep blinking at this interval
int led_avionicState = LOW;
unsigned long led_avionicPreviousMillis = 0;

//-----------Variáveis de recuperação---------------//
float lastAltitude;

// An array of pointers. 17 variables of 4 bytes.
uint8_t *rf_radioPacket[RF_BYTES_IN_PACKET] =
{
    (uint8_t*) & RF_VALIDATION_HEADER,
    (uint8_t*) & rf_packetID,
    (uint8_t*) & rf_packetTime,
    (uint8_t*) & gps_lat,
    (uint8_t*) & gps_lon,
    (uint8_t*) & imu_struct.magnetometro[0],
    (uint8_t*) & imu_struct.magnetometro[1],
    (uint8_t*) & imu_struct.magnetometro[2],
    (uint8_t*) & imu_struct.acelerometro[0],
    (uint8_t*) & imu_struct.acelerometro[1],
    (uint8_t*) & imu_struct.acelerometro[2],
    (uint8_t*) & imu_struct.giroscopio[0],
    (uint8_t*) & imu_struct.giroscopio[1],
    (uint8_t*) & imu_struct.giroscopio[2],
    (uint8_t*) & imu_struct.barometro[0], // pressure
    (uint8_t*) & imu_struct.barometro[1], // altitude
    (uint8_t*) & imu_struct.barometro[2]  // temperature
};

//---------------Função GPS Venus---------------//
void getField(char* buffer, int index)
{
    int sentencePos = 0;
    int fieldPos = 0;
    int commaCount = 0;
    while (sentencePos < GPS_SENTENCE_SIZE)
    {
        if (gps_sentence[sentencePos] == ',')
        {
            commaCount ++;
            sentencePos ++;
        }
        if (commaCount == index)
        {
            buffer[fieldPos] = gps_sentence[sentencePos];
            fieldPos ++;
        }
        sentencePos ++;
    }
buffer[fieldPos] = '\0';
}



void setup()
{
    int generalCounter;
    #if DEBUG_SERIAL
        Serial.begin(9600); //Initialize Serial Port at 9600 baudrate.
    #endif
    Serial4.begin(9600); //Initialize GPS port at 9600 baudrate.
    pinMode(PIN_LED_RECOVERY, OUTPUT);
    pinMode(PIN_LED_AVIONIC, OUTPUT);
    pinMode(PIN_LED_ALL_OK_AND_RF, OUTPUT);

//---------------Setup LORA---------------//
    pinMode(PIN_RFM95_RST, OUTPUT);
    digitalWrite(PIN_RFM95_RST, HIGH);

    delay(100);
    digitalWrite(PIN_RFM95_RST, LOW);
    delay(10);
    digitalWrite(PIN_RFM95_RST, HIGH);
    delay(10);

    generalCounter = 0;
    while (!rf95.init() and (generalCounter++ < RF_INIT_MAX_TRIES))
    {
        #if DEBUG_SERIAL
            Serial.println("LoRa nao inicializou");
            Serial.println("Realizando nova tentativa...");
        #endif
    }
    #if DEBUG_SERIAL
        Serial.println("LoRa inicializado!");
    #endif

    if (!rf95.setFrequency(RF95_FREQ))
    {
        #if DEBUG_SERIAL
            Serial.println("setFrequency falhou!");
        #endif

    }

    rf95.setTxPower(23, false);
//END of Setup LORA-----------------------//

//---------------Setup Modulo SD---------------//
    if (!SD.begin(chipSelect))
    {
        #if DEBUG_SERIAL
            Serial.println("Inicialização do modulo SD falhou!");
        #endif
        sdIsWorking = 0;
    }
    #if DEBUG_SERIAL
        Serial.println("Inicialização do modulo SD concluída.");
    #endif

    if (sdIsWorking)
    {
        fileLog = SD.open("DADOSMVP.txt", FILE_WRITE);
        if (fileLog)
        {
            fileLog.println("sep =, "); //This line handles Excel CSV configuration.
            fileLog.println("Tempo(ms), Pressão (hPa), Altitude (m), Temperatura (°C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s), Lat, Long");
        }
        else
        {
            digitalWrite(PIN_LED_RECOVERY, HIGH);
        }
        fileLog.close();
    }
//END of Setup Modulo SD--------------------------------//


//---------------Setup Serial Monitor-------------------//
    #if DEBUG_SERIAL
        Serial.println("Minerva Rockets - UFRJ");
        Serial.println("sep =, "); //This line handles Excel CSV configuration.
        Serial.println("Tempo(ms), Pressão (hPa), Altitude (m), Temperatura (°C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s), Lat, Long");
    #endif

//-------------- IMU's Init ----------------------------//
    InitBMP(); //Initialize BMP module
    InitAcel(); //Initialize Accelerometer module
    InitGyro(); //Initialize Gyroscope module
    InitMag(); //Initialize Magnetometer module
    //digitalWrite(PIN_LED_AVIONIC, HIGH);

//------------Setup Buzzer -----------------------------//
    pinMode(BUZZER_PIN,OUTPUT);

//END of Setup  ---------------------------------------------------------------------------------------------------------//
}









void loop()
{

    if(millis() - led_avionicPreviousMillis > LED_AVIONIC_INTERVAL)
    {
        digitalWrite(PIN_LED_AVIONIC, (led_avionicState = !led_avionicState)); // set the LED with the led_avionicState of the variable:
        led_avionicPreviousMillis = millis(); // save the last time you blinked the LED
    }

    //---------------IMU structure definition---------------//
    GetBMP(imu_pstruct); //Fills the BMP085 module information into the IMU structure
    GetAcel(imu_pstruct); //Fills the Accelerometer module information into the IMU structure
    GetGyro(imu_pstruct); //Fills the Gyroscope module information into the IMU structure
    GetMag(imu_pstruct); //Fills the Magnetometer module information into the IMU structure


    //---------------Recuperação---------------//
    if (((abs(imu_struct.acelerometro[0]) < 1) && (abs(imu_struct.acelerometro[1]) < 1) && (abs(imu_struct.acelerometro[2]) < 1)))
    {
        digitalWrite(PIN_LED_RECOVERY, HIGH);
        if (sdIsWorking)
        {
            fileLog = SD.open("DADOSMVP.txt", FILE_WRITE);
            if (fileLog)
            {
                fileLog.print(millis());fileLog.print(" ,");
                fileLog.print(imu_struct.acelerometro[2]);fileLog.print(" ,");
                fileLog.println("Recovery opened");
            }
            fileLog.close();
        }
    }

    //---------------GPS Venus---------------//

    static int i = 0;
    if (Serial4.available())
    {
        while (true)
        {
            char ch = Serial4.read();
            if (ch != '\n' && i < GPS_SENTENCE_SIZE)
            {
                gps_sentence[i] = ch;
                i++;
            }
            else
            {
                gps_sentence[i] = '\0';
                i = 0;
                getField(gps_stringBuffer, 0);
                if (strcmp(gps_stringBuffer, "$GPRMC") == 0)
                {
                    float gps_lat, gps_lon;

                    #if DEBUG_SERIAL
                        Serial.print("Lat: ");
                    #endif

                    getField(gps_stringBuffer, 3);
                    gps_lat = String(gps_stringBuffer).toFloat();

                    #if DEBUG_SERIAL
                        Serial.print(gps_stringBuffer);
                    #endif

                    getField(gps_stringBuffer, 4);
                    c_lat = gps_stringBuffer[0];
                    #if DEBUG_SERIAL
                        Serial.print(gps_stringBuffer);
                        Serial.print(" Long: ");
                    #endif

                    getField(gps_stringBuffer, 5);
                    gps_lon = String(gps_stringBuffer).toFloat();

                    #if DEBUG_SERIAL
                        Serial.print(gps_stringBuffer);
                    #endif

                    getField(gps_stringBuffer, 6);
                    c_lon = gps_stringBuffer[0];

                    #if DEBUG_SERIAL
                        Serial.println(gps_stringBuffer);
                    #endif
                }
                break;
            }
        }
    }



//---------------SD Logging Code---------------//
    if (sdIsWorking)
    {
        fileLog = SD.open("DADOSMVP.txt", FILE_WRITE);
        if (fileLog)
        {
            fileLog.print(millis());fileLog.print(" ,");
            fileLog.print(imu_struct.barometro[0]);fileLog.print(" ,");
            fileLog.print(imu_struct.barometro[1]);fileLog.print(" ,");
            fileLog.print(imu_struct.barometro[2]);fileLog.print(" ,");
            fileLog.print(imu_struct.acelerometro[0]);fileLog.print(" ,");
            fileLog.print(imu_struct.acelerometro[1]);fileLog.print(" ,");
            fileLog.print(imu_struct.acelerometro[2]);fileLog.print(" ,");
            fileLog.print(imu_struct.giroscopio[0]);fileLog.print(" ,");
            fileLog.print(imu_struct.giroscopio[1]);fileLog.print(" ,");
            fileLog.print(imu_struct.giroscopio[2]);fileLog.print(" ,");
            fileLog.print(imu_struct.magnetometro[0]);fileLog.print(" ,");
            fileLog.print(imu_struct.magnetometro[1]);fileLog.print(" ,");
            fileLog.print(imu_struct.magnetometro[2]);fileLog.print(" ,");
            fileLog.print(gps_lat);fileLog.print("");
            fileLog.print(gps_lon);fileLog.print("");
            fileLog.close();

        }
    }
//---------------Code for serial debugging---------------//
    #if DEBUG_SERIAL
        Serial.print(millis());Serial.print(" ,");
        Serial.print(imu_struct.barometro[0]);Serial.print(" ,");
        Serial.print(imu_struct.barometro[1]);Serial.print(" ,");
        Serial.print(imu_struct.barometro[2]);Serial.print(" ,");
        Serial.print(imu_struct.acelerometro[0]);Serial.print(" ,");
        Serial.print(imu_struct.acelerometro[1]);Serial.print(" ,");
        Serial.print(imu_struct.acelerometro[2]);Serial.print(" ,");
        Serial.print(imu_struct.giroscopio[0]);Serial.print(" ,");
        Serial.print(imu_struct.giroscopio[1]);Serial.print(" ,");
        Serial.print(imu_struct.giroscopio[2]);Serial.print(" ,");
        Serial.print(imu_struct.magnetometro[0]);Serial.print(" ,");
        Serial.print(imu_struct.magnetometro[1]);Serial.print(" ,");
        Serial.println(imu_struct.magnetometro[2]);
        // Serial.print(gps_lat);Serial.print("");
        // Serial.print(gps_lon);Serial.print("");
        // Serial.println(c_lon);
    #endif
    lastAltitude = imu_struct.barometro[2];


//----------------Buzzer------------------------//
    #if BUZZER_ACTIVATE
        if ((millis() - buzzer_lastTime > 1000 * (1 / BUZZER_OK_FREQ)))
        {
            digitalWrite(BUZZER_PIN, (buzzer_status = !buzzer_status));
            buzzer_lastTime = millis();
        }
    #endif

//---------------Envio de pacotes do LORA---------------//
    rf_packetTime = millis()/1000; // Packet time, in seconds.
    rf_packetID ++;
    rf95.sendArrayOfPointersOf4Bytes(rf_radioPacket, RF_WORDS_IN_PACKET);
}
