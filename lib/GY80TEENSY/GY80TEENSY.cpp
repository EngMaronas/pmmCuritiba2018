// GY-80 IMU Library especially written for use with Teensy 3.6
// Code written by Marcelo Maronas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - February 19, 2018
// Edited by Henrique Bruno @ Minerva Rockets - April 14, 2018
// Using Adafruit Libraries.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include <GY80TEENSY.h>
#define GY80_DEBUG_SERIAL false

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12346);
L3G gyro;

sensors_event_t event;

void GetGyro(IMU_s *imu) {
    gyro.read();

    imu->giroscopio[0] = gyro.g.x;
    imu->giroscopio[1] = gyro.g.y;
    imu->giroscopio[2] = gyro.g.z;
}

void GetAcel(IMU_s *imu) {
    accel.getEvent(&event);

    imu->acelerometro[0] = event.acceleration.x;
    imu->acelerometro[1] = event.acceleration.y;
    imu->acelerometro[2] = event.acceleration.z;
}

void GetMag(IMU_s *imu) {
    mag.getEvent(&event);

    imu->magnetometro[0] = event.magnetic.x;
    imu->magnetometro[1] = event.magnetic.y;
    imu->magnetometro[2] = event.magnetic.z;
}

void GetBMP(IMU_s *imu) {
    bmp.getEvent(&event);
    bmp.getTemperature(&(imu->barometro[2]));
    imu->barometro[0] = event.pressure;
    imu->barometro[1] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure);
}

int InitAcel() //ADXL45 SETUP
{
    if(!accel.begin()) /* Initialise the sensor */
    {
        #if GY80_DEBUG_SERIAL
            /* There was a problem detecting the ADXL345 ... check your connections */
            Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        #endif

        return 1;
    }
    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL345_RANGE_16_G);
    return 0;
    //ADXL45 SETUP END
}



int InitGyro() //L2G4200D Setup
{
    if (!gyro.init())
    {
        #if GY80_DEBUG_SERIAL
            Serial.println("Failed to autodetect gyro type!");
        #endif

        return 1;
    }
    gyro.enableDefault();
    return 0;
}



int InitMag() //HMC5884 Setup
{
    if(!mag.begin()) // Initialise the sensor
    {
        #if GY80_DEBUG_SERIAL
            /* There was a problem detecting the HMC5883 ... check your connections */
            Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        #endif

        return 1;
    }
    return 0;
}



int InitBMP()  //BMP085 Setup
{
    if(!bmp.begin())
    {
        #if GY80_DEBUG_SERIAL
            /* There was a problem detecting the BMP085 ... check your connections */
            Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
        #endif

        return 1;
    }
    return 0;
}
