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
 *  * Caso queria usar outra entrada RX/TX consultar pinagem em: https://www.pjrc.com/teensy/pinout.html
 * E trocar Serial2 pelo numero do Serial a utilizar
 */

//---------------Inclusão de bibliotecas---------------//
#include <GY80TEENSY.h>
#include <RH_RF95.h>
#include <SD.h>
#include <SPI.h>

//---------------Definições Lora-------------------//
#define RFM95_CS 15
#define RFM95_RST 17
#define RFM95_INT 16
#define RF95_FREQ 915.0
#define RF_WORDS_IN_PACKET 16
#define RF_BYTES_IN_PACKET (WORDS_IN_PACKET * 4)
RH_RF95 rf95(RFM95_CS, RFM95_INT);



//---------------Variáveis GPS Venus---------------//
const int sentenceSize = 80;
bool gpsRead;
char sentence[sentenceSize];
float lat, lon;
char c_lat, c_lon;

//---------------Função GPS Venus---------------//
void getField(char* buffer, int index)
{
    int sentencePos = 0;
    int fieldPos = 0;
    int commaCount = 0;
    while (sentencePos < sentenceSize)
    {
        if (sentence[sentencePos] == ',')
        {
            commaCount ++;
            sentencePos ++;
        }
        if (commaCount == index)
        {
            buffer[fieldPos] = sentence[sentencePos];
            fieldPos ++;
        }
        sentencePos ++;
    }
buffer[fieldPos] = '\0';
}


//Variable definition is within the library, those are the sensors used.The library needs to be updated for multiples GY80 use.
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12346);
//L3G gyro;

//-------------- Definições Buzzer ----------------//
#define BUZZER_ACTIVATE 1
#define BUZZER_PIN  2
#define BUZZER_FREQ 100
long t_anterior=0;
int buzzer_estado = LOW;
//---------------Variáveis módulo SD---------------//
File myFile;
const int chipSelect = BUILTIN_SDCARD;


//--------------Declaração do Struct---------------//
IMU_s struct_imu;
IMU_s *pstruct_imu = &struct_imu;


//--------------Variáveis para Debug---------------//
bool DebugSerial = 1; //Prints the values stored in the structure IMU_s
bool sdLog = 1; //Prints the values of IMU_s in SD card


//---------------Modifica o tempo com que as informações aparecem no Monitor---------------//
float Delay_Time = 500;

//---------------Definições dos LEDS---------------//
int AvionicLed = 3;
int RecoveryLed = 1;
int SdRecording = 4;
int ledState = LOW;
long previousMillis = 0;
#define INTERVAL 100

//-----------Variáveis de recuperação---------------//
float LastAltitude;


void setup() {
    Serial.begin(9600); //Initialize Serial Port at 9600 baudrate.
    Serial4.begin(9600); //Initialize GPS port at 9600 baudrate.
    pinMode(RecoveryLed, OUTPUT);


    //---------------Setup LORA---------------//
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    delay(100);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init())
    {
        Serial.println("LoRa nao inicializou");
        Serial.println("Realizando nova tentativa...");
    }
    Serial.println("LoRa inicializado!");

    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println("setFrequency falhou!");
        while (1);
    }

    rf95.setTxPower(23, false);


    //---------------Setup Modulo SD---------------//
    pinMode(SdRecording, OUTPUT);
    pinMode(AvionicLed, OUTPUT);
    if (!SD.begin(chipSelect))
    {
        Serial.println("Inicialização do modulo SD falhou!");
        sdLog = 0;
    }
    Serial.println("Inicialização do modulo SD concluída.");
    if (sdLog)
    {
        myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
        if (myFile)
        {
            myFile.println("sep =, "); //This line handles Excel CSV configuration.
            myFile.println("Tempo(ms), Pressão (hPa), Altitude (m), Temperatura (°C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s), Lat, Long");
        }
        else
        {
            digitalWrite(RecoveryLed, HIGH);
        }
        myFile.close();
    }

    //---------------Setup Serial Monitor---------------//
    Serial.println("Minerva Rockets - UFRJ");
    Serial.println("sep =, "); //This line handles Excel CSV configuration.
    Serial.println("Tempo(ms), Pressão (hPa), Altitude (m), Temperatura (°C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s), Lat, Long");

    //--------------Inicialização IMU's-----------------//
    InitBMP(); //Initialize BMP module
    InitAcel(); //Initialize Accelerometer module
    InitGyro(); //Initialize Gyroscope module
    InitMag(); //Initialize Magnetometer module
    //digitalWrite(AvionicLed, HIGH);

    //------------Setup Buzzer -------------------//
    pinMode(BUZZER_PIN,OUTPUT);

}

void loop() {
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > INTERVAL)
    {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
            ledState = HIGH;
        else
            ledState = LOW;

        // set the LED with the ledState of the variable:
        digitalWrite(AvionicLed, ledState);
    }
    //---------------IMU structure definition---------------//
    GetBMP(pstruct_imu); //Fills the BMP085 module information into the IMU structure
    GetAcel(pstruct_imu); //Fills the Accelerometer module information into the IMU structure
    GetGyro(pstruct_imu); //Fills the Gyroscope module information into the IMU structure
    GetMag(pstruct_imu); //Fills the Magnetometer module information into the IMU structure


    //---------------Recuperação---------------//
    if (((abs(struct_imu.acelerometro[0]) < 1) && (abs(struct_imu.acelerometro[1]) < 1) && (abs(struct_imu.acelerometro[2]) < 1)))
    {
        digitalWrite(RecoveryLed, HIGH);
        if (sdLog)
        {
            myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
            if (myFile)
            {
                myFile.print(millis());myFile.print(" ,");
                myFile.print(struct_imu.acelerometro[2]);myFile.print(" ,");
                myFile.println("Recovery opened");
            }
            myFile.close();
        }
    }

    //---------------GPS Venus---------------//

    char field[20];
    static int i = 0;
    if (Serial4.available())
    {
        while (true)
        {
            char ch = Serial4.read();
            if (ch != '\n' && i < sentenceSize)
            {
                sentence[i] = ch;
                i++;
            }
            else
            {
                sentence[i] = '\0';
                i = 0;
                char field[20];
                getField(field, 0);
                if (strcmp(field, "$GPRMC") == 0)
                {
                    float lat, lon;
                    char c_lat, c_lon;
                    Serial.print("Lat: ");
                    getField(field, 3);
                    lat = String(field).toFloat();
                    Serial.print(field);
                    getField(field, 4);
                    c_lat = field[0];
                    Serial.print(field);

                    Serial.print(" Long: ");
                    getField(field, 5);
                    lon = String(field).toFloat();
                    Serial.print(field);
                    getField(field, 6);
                    c_lon = field[0];
                    Serial.println(field);
                }
                break;
            }
        }
    }



    //---------------SD Logging Code---------------//
    if (sdLog)
    {
        myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
        digitalWrite(SdRecording, LOW);
        if (myFile)
        {
            myFile.print(millis());myFile.print(" ,");
            myFile.print(struct_imu.barometro[0]);myFile.print(" ,");
            myFile.print(struct_imu.barometro[1]);myFile.print(" ,");
            myFile.print(struct_imu.barometro[2]);myFile.print(" ,");
            myFile.print(struct_imu.acelerometro[0]);myFile.print(" ,");
            myFile.print(struct_imu.acelerometro[1]);myFile.print(" ,");
            myFile.print(struct_imu.acelerometro[2]);myFile.print(" ,");
            myFile.print(struct_imu.giroscopio[0]);myFile.print(" ,");
            myFile.print(struct_imu.giroscopio[1]);myFile.print(" ,");
            myFile.print(struct_imu.giroscopio[2]);myFile.print(" ,");
            myFile.print(struct_imu.magnetometro[0]);myFile.print(" ,");
            myFile.print(struct_imu.magnetometro[1]);myFile.print(" ,");
            myFile.print(struct_imu.magnetometro[2]);myFile.print(" ,");
            myFile.print(lat);myFile.print("");
            myFile.print(c_lat);myFile.print(" ,");
            myFile.print(lon);myFile.print("");
            myFile.println(c_lon);

        }
        digitalWrite(SdRecording, HIGH);
        myFile.close();
    }
    //---------------Code for serial debugging---------------//
    if (DebugSerial)
    {
        char field[20];
        Serial.print(millis());Serial.print(" ,");
        Serial.print(struct_imu.barometro[0]);Serial.print(" ,");
        Serial.print(struct_imu.barometro[1]);Serial.print(" ,");
        Serial.print(struct_imu.barometro[2]);Serial.print(" ,");
        Serial.print(struct_imu.acelerometro[0]);Serial.print(" ,");
        Serial.print(struct_imu.acelerometro[1]);Serial.print(" ,");
        Serial.print(struct_imu.acelerometro[2]);Serial.print(" ,");
        Serial.print(struct_imu.giroscopio[0]);Serial.print(" ,");
        Serial.print(struct_imu.giroscopio[1]);Serial.print(" ,");
        Serial.print(struct_imu.giroscopio[2]);Serial.print(" ,");
        Serial.print(struct_imu.magnetometro[0]);Serial.print(" ,");
        Serial.print(struct_imu.magnetometro[1]);Serial.print(" ,");
        Serial.println(struct_imu.magnetometro[2]);
        // Serial.print(lat);Serial.print("");
        // Serial.print(c_lat);Serial.print(" ,");
        // Serial.print(lon);Serial.print("");
        // Serial.println(c_lon);
    }
    LastAltitude = struct_imu.barometro[2];


    //---------------Envio de pacotes do LORA---------------//
    const char RF_VALIDATION_HEADER[4] = {'M', 'N', 'R', 'V'} // Make sure that there is no NULL-terminator char.
    uint8_t* radiopacket[RF_BYTES_IN_PACKET];

    // An array of pointers.
    uint8_t *radiopacket[RF_BYTES_IN_PACKET] =
    {
        &(uint8_t) RF_VALIDATION_HEADER,
        &(uint8_t) lat,
        &(uint8_t) lon,
        &(uint8_t) struct_imu.barometro[0],
        &(uint8_t) struct_imu.barometro[1],
        &(uint8_t) struct_imu.barometro[2],
        &(uint8_t) struct_imu.barometro[3],
        &(uint8_t) struct_imu.acelerometro[0],
        &(uint8_t) struct_imu.acelerometro[1],
        &(uint8_t) struct_imu.acelerometro[2],
        &(uint8_t) struct_imu.giroscopio[0],
        &(uint8_t) struct_imu.giroscopio[1],
        &(uint8_t) struct_imu.giroscopio[2],
        &(uint8_t) struct_imu.magnetometro[0],
        &(uint8_t) struct_imu.magnetometro[1],
        &(uint8_t) struct_imu.magnetometro[2]
    }
    rf95.send(radiopacket, RF_BYTES_IN_PACKET);

    // Espera o pacote ser enviando



    //----------------Buzzer------------------------//

    #if BUZZER_ACTIVATE
        if ((millis() - t_anterior > 1000 * (1 / BUZZER_FREQ)))
        {
            digitalWrite(BUZZER_PIN,not(buzzer_estado));
            buzzer_estado = not(buzzer_estado);
            t_anterior = millis();
        }
    #endif

}
