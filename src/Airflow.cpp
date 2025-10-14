#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include "Airflow.h"
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

Airflow::Airflow (float inletSize, float throatSize, float airdensity)  
                : inletDiameter(inletSize), throatDiameter(throatSize), airDensity(airdensity) {}

void Airflow::airflowSetup(int sda,int scl, int address){
    Serial.begin(9600);
     if (!ads.begin()) {
    Serial.println("Failed to connect to sensor");
    while (1);
  }
}

void Airflow::getData(){
    this->voltage = ads.readADC_SingleEnded(0);
}

void Airflow::convertVolttoPressure (){
     float voltage = this->voltage;

}

void Airflow::bernoulli(){
  float DISCHARGE_COEFFICIENT = 0.98;
  float inletDiameter = this->inletDiameter;
  float throatDiameter = this->throatDiameter;
  float inletArea = M_PI * pow((inletDiameter / 2.0), 2);
  float throatArea = M_PI * pow((throatDiameter / 2.0), 2);
  float flowRate = DISCHARGE_COEFFICIENT * throatArea *
                     sqrt((2.0 * deltaPressure) / (this->airDensity * (1.0 - pow((throatArea / inletArea), 2))));

   this->airFlow = flowRate;
}

float Airflow::getAirflow(){
  return this->airFlow;
}