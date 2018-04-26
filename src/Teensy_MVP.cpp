/*  Code written by Marcelo Maroñas, Eduardo Alves, Lucas Ribeiro, HENRIQUE BRUNO and Victor de Lucca @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - March 06, 2018
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
// https://forum.pjrc.com/threads/39158-Using-SdFat-to-acces-Teensy-3-6-SD-internal-card-(-amp-with-audio-board)
//---------------Inclusão de bibliotecas---------------//
#include <GY80TEENSY.h>
#include <RH_RF95.h>
#include <pmmConsts.h>
#include <pmmGps.h>
#include <pmmSd.h>
#include <pmmErrorsAndSignals.h>

//--------------- Time delay -----------------//
unsigned long nextMillis_barometer = 0, nextMillis_magnetometer = 0, nextMillis_accelerometer = 0, nextMillis_gyroscope = 0, nextMillis_rf = 0;
//--------------- Errors? --------------------//
int sdIsWorking = 1, rfIsWorking = 1,
    accelIsWorking = 1, gyroIsWorking = 1, magnIsWorking = 1, baroIsWorking = 1;

PmmErrorsAndSignals pmmErrorsAndSignals;

//--------------- LoRa vars ------------------//
float packetTimeFloatS = 0, packetIDfloat = 0;
unsigned long packetTimeMs = 0, packetIDul = 0;
RH_RF95 rf95(PIN_RFM95_CS, PIN_RFM95_INT);

//--------------- GPS Venus Vars---------------//
GpsManager gpsManager;
Gps_structType gps_struct;

//--------------- SD vars ------------------//
SdManager sdManager;
char logString[LOG_BUFFER_LENGTH];
int32_t logStringLength;
char SD_LOG_HEADER[] = {"sep =, \nPacketID, Time(ms), Latitude, Longitude, Pressure (hPa), Altitude (m), Temperature (C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s)"}; //This line handles Excel CSV configuration.

//------------ IMU Struct Declaration ------------//
IMU_s imu_struct;
IMU_s *imu_pstruct = &imu_struct;

//---------------- Recuperation ------------------//
float lastAltitude;

// An array of pointers. 17 variables of 4 bytes.

uint8_t *rf_radioPacket[RF_BYTES_IN_PACKET] =
{
    (uint8_t*) & RF_VALIDATION_HEADER,
    (uint8_t*) & packetIDfloat,
    (uint8_t*) & packetTimeFloatS,
    (uint8_t*) & gps_struct.latitude,
    (uint8_t*) & gps_struct.longitude,
    (uint8_t*) & gps_struct.altitude,
    (uint8_t*) & gps_struct.horizontalSpeed,
    (uint8_t*) & gps_struct.speedNorth,
    (uint8_t*) & gps_struct.speedEast,
    (uint8_t*) & gps_struct.speedDown,
    (uint8_t*) & gps_struct.headingDegree,
    (uint8_t*) & gps_struct.satellites,
    (uint8_t*) & imu_struct.barometro[0], // pressure
    (uint8_t*) & imu_struct.barometro[1], // altitude
    (uint8_t*) & imu_struct.barometro[2], // temperature
    (uint8_t*) & imu_struct.acelerometro[0],
    (uint8_t*) & imu_struct.acelerometro[1],
    (uint8_t*) & imu_struct.acelerometro[2],
    (uint8_t*) & imu_struct.giroscopio[0],
    (uint8_t*) & imu_struct.giroscopio[1],
    (uint8_t*) & imu_struct.giroscopio[2],
    (uint8_t*) & imu_struct.magnetometro[0],
    (uint8_t*) & imu_struct.magnetometro[1],
    (uint8_t*) & imu_struct.magnetometro[2]
};

unsigned long timePrint;

void setup()
{
    #if DEBUG_SERIAL
        Serial.begin(250000); //Initialize Serial Port at 9600 baudrate.
        while (!Serial); // wait for serial port to connect. Needed for native USB port only
    #endif

// SETUP SD //
    int fileId = 0;
    if (sdManager.init())
    {
        DEBUG_PRINT("Inicialização do modulo SD falhou!");
        sdIsWorking = 0;
    }
    else
    {
        DEBUG_PRINT("Inicialização do modulo SD concluída.");
        fileId = sdManager.setFilenameAutoId(FILENAME_BASE_PREFIX, FILENAME_BASE_SUFFIX);
        #if DEBUG_SERIAL
            char tempFilename[FILENAME_MAX_LENGTH];
            sdManager.getFilename(tempFilename, FILENAME_MAX_LENGTH);
            Serial.print("Filename is = \""); Serial.print(tempFilename); Serial.println("\"");
        #endif
    }

// SETUP SD END //

    pmmErrorsAndSignals.init(&rf95, fileId);

// ---- GPS
    gpsManager.init();

//---------------Setup LORA---------------//
    pinMode(PIN_RFM95_RST, OUTPUT);
    digitalWrite(PIN_RFM95_RST, HIGH);

    delay(100); digitalWrite(PIN_RFM95_RST, LOW); delay(10); digitalWrite(PIN_RFM95_RST, HIGH); delay(10);

    int rf_initCounter = 0;
    while (!(rfIsWorking = rf95.init()) and (rf_initCounter++ < RF_INIT_MAX_TRIES))
    {
        #if DEBUG_SERIAL
            Serial.print("LoRa nao inicializou, tentativa n"); Serial.println(rf_initCounter);
            Serial.println("Realizando nova tentativa...");
        #endif
    }

    if (!rfIsWorking)
        pmmErrorsAndSignals.reportError(ERROR_RF_INIT, 0, sdIsWorking, rfIsWorking);

    else // if RF is working
    {
        if (!(rfIsWorking = rf95.setFrequency(RF95_FREQ)))
        {
            DEBUG_PRINT("setFrequency falhou!");

            pmmErrorsAndSignals.reportError(ERROR_RF_SET_FREQ, 0, sdIsWorking, rfIsWorking);
        }
        else // if RF is working
        {
            rf95.setTxPower(23, false);
            DEBUG_PRINT("LoRa inicializado!");

        }
    }
//END of Setup LORA-----------------------//

//---------------Setup Modulo SD---------------//

    if (sdIsWorking) // This conditional exists so you can disable sd writing by changing the initial sdIsWorking value on the variable declaration.
    {
        if (sdManager.writeToFile(SD_LOG_HEADER, strlen(SD_LOG_HEADER)))
        {
            DEBUG_PRINT("sdIsWorking = False");
            sdIsWorking = 0;
            pmmErrorsAndSignals.reportError(ERROR_SD, 0, sdIsWorking, rfIsWorking);
        }
        else
        {
            DEBUG_PRINT("sdIsWorking = True");
        }
    }
//END of Setup Modulo SD--------------------------------//

    DEBUG_PRINT("\nMinerva Rockets - UFRJ");
    DEBUG_PRINT("PacketID, Time(ms), Latitude, Longitude, Pressure (hPa), Altitude (m), Temperature (C), AcelX (m/s²), AcelY (m/s²), AcelZ (m/s²), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s), MagnetoX (T/s), MagnetoY (T/s), MagnetoZ (T/s)");


//-------------- IMU's Init ----------------------------//
    if (InitBMP())
    {
        baroIsWorking = 0;
        DEBUG_PRINT("BAROMETER INIT ERROR");
        pmmErrorsAndSignals.reportError(ERROR_BAROMETER_INIT, 0, sdIsWorking, rfIsWorking);
    }
    if (InitAcel())
    {
        accelIsWorking = 0;
        DEBUG_PRINT("ACCELEROMETER INIT ERROR");
        pmmErrorsAndSignals.reportError(ERROR_ACCELEROMETER_INIT, 0, sdIsWorking, rfIsWorking);
    }
    if (InitGyro())
    {
        gyroIsWorking = 0;
        DEBUG_PRINT("GYROSCOPE INIT ERROR");
        pmmErrorsAndSignals.reportError(ERROR_GYROSCOPE_INIT, 0, sdIsWorking, rfIsWorking);
    }
    if (InitMag())
    {
        magnIsWorking = 0;
        DEBUG_PRINT("MAGNETOMETER INIT ERROR");
        pmmErrorsAndSignals.reportError(ERROR_MAGNETOMETER_INIT, 0, sdIsWorking, rfIsWorking);
    }

//END of Setup  ---------------------------------------------------------------------------------------------------------//
}



void loop()
{
    //DEBUG_MAINLOOP_PRINT(0);
    packetTimeMs = millis();                  // Packet time, in miliseconds. (unsigned long)
    packetTimeFloatS = packetTimeMs / 1000.0; // Packet time, in seconds. (float)
    DEBUG_MAINLOOP_PRINT(1);
    //---------------IMU structure definition---------------//

    if (millis() >= nextMillis_barometer)
    {
        nextMillis_barometer = millis() + DELAY_MS_BAROMETER;
        if (baroIsWorking)
            GetBMP(imu_pstruct); //Fills the BMP085 module information into the IMU structure
    }
    DEBUG_MAINLOOP_PRINT(2);

    if (accelIsWorking)
        GetAcel(imu_pstruct); //Fills the Accelerometer module information into the IMU structure
    DEBUG_MAINLOOP_PRINT(3);
    if (gyroIsWorking)
        GetGyro(imu_pstruct); //Fills the Gyroscope module information into the IMU structure


    DEBUG_MAINLOOP_PRINT(4);
    if (magnIsWorking)
        GetMag(imu_pstruct); //Fills the Magnetometer module information into the IMU structure
    DEBUG_MAINLOOP_PRINT(5);

    //---------------Recuperação---------------//
    if (((abs(imu_struct.acelerometro[0]) < 1) && (abs(imu_struct.acelerometro[1]) < 1) && (abs(imu_struct.acelerometro[2]) < 1)))
    {
        DEBUG_PRINT("RECUPERATION!");
        pmmErrorsAndSignals.reportRecuperation(packetIDul, sdIsWorking, rfIsWorking);
    }

    //---------------GPS Venus---------------//

    DEBUG_MAINLOOP_PRINT(6);
    gpsManager.updateIfAvailable(&gps_struct);

    DEBUG_MAINLOOP_PRINT(7);
//---------------Code for serial debugging---------------//
//gps_struct.latitude,        gps_struct.longitude
    logStringLength = snprintf(logString, LOG_BUFFER_LENGTH, "%lu ,%lu ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f\n",
        packetIDul,                 packetTimeMs,               gps_struct.latitude,        gps_struct.longitude,       gps_struct.altitude,
        gps_struct.horizontalSpeed, gps_struct.speedNorth,      gps_struct.speedEast,       gps_struct.speedDown,       gps_struct.headingDegree,
        gps_struct.satellites,      imu_struct.barometro[0],    imu_struct.barometro[1],    imu_struct.barometro[2],    imu_struct.acelerometro[0],
        imu_struct.acelerometro[1], imu_struct.acelerometro[2], imu_struct.giroscopio[0],   imu_struct.giroscopio[1],   imu_struct.giroscopio[2],
        imu_struct.magnetometro[0], imu_struct.magnetometro[1], imu_struct.magnetometro[2]);

    #if DEBUG_SERIAL
        Serial.print(logString);
    #endif

    DEBUG_MAINLOOP_PRINT(8);
//---------------SD Logging Code---------------//

    if (sdIsWorking)
    {
        DEBUG_MAINLOOP_PRINT(8.1);
        if (sdManager.writeToFile(logString, logStringLength))
        {
            DEBUG_PRINT("SD WRITING ERROR!");
            sdIsWorking = 0;
            pmmErrorsAndSignals.reportError(ERROR_SD_WRITE, packetIDul, sdIsWorking, rfIsWorking);
        }
    }
    else
    {

    }
    DEBUG_MAINLOOP_PRINT(9);
//-------------- Send RF package ---------------//
    if (millis() >= nextMillis_rf)
    {
        nextMillis_rf = millis() + DELAY_MS_RF;
        if (rfIsWorking)
        {
            pmmErrorsAndSignals.blinkRfLED(HIGH);
            rf95.sendArrayOfPointersOf4Bytes(rf_radioPacket, RF_WORDS_IN_PACKET);
            pmmErrorsAndSignals.blinkRfLED(LOW);
        }
    }
    DEBUG_MAINLOOP_PRINT(10);
    pmmErrorsAndSignals.updateLedsAndBuzzer();
    lastAltitude = imu_struct.barometro[2];
    packetIDfloat ++;
    packetIDul ++;
    /*if (packetIDul % 100 == 0)
    {
        Serial.print("timeMsBetween 100 cycles = "); Serial.println(millis() - timePrint);
        timePrint = millis();
    }*/
}
