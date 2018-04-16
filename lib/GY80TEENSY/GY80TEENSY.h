// GY-80 IMU Library especially written for use with Teensy 3.6
// Code written by Marcelo Maronas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - February 19, 2018
// Using Adafruit Libraries.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_ADXL345_U.h>
#include <L3G.h>
#include <Adafruit_HMC5883_U.h>

typedef struct
{
    float acelerometro[3]; //Posicoes 1,2,3, respectivamente sao as Aceleracoes em x,y,z
    int magnetometro[3]; //Posicoes 1,2,3, respectivamente sao as Campos Magneticos em x,y,z
    int giroscopio[3]; //Posicoes 1, 2, 3, respectivamente sao a velocidade angular em x,y,z
    double barometro[3]; //Posicoes 1,2,3 respectivamente sao Pressao, Altura e Temperatura
}IMU_s; //IMU Structure

//Functions headers
int GetGyro(IMU_s *imu);

int GetAcel(IMU_s *imu);

int GetMag(IMU_s *imu);

int GetBMP(IMU_s *imu);

int InitAcel();

int InitGyro();

int InitMag();

int InitBMP();

//Functions headers
