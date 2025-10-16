#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include "Airflow.h"
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

Airflow::Airflow(float inletSize, float throatSize, float airdensity)
    : inletDiameter(inletSize), throatDiameter(throatSize), airDensity(airdensity) {}

void Airflow::airflowSetup(int sda, int scl, int address) {
  Serial.begin(115200);
  Wire.begin(sda, scl);

  if (!ads.begin(address)) {
    Serial.println("Failed to connect to ADS1115");
    while (1);
  }

  ads.setGain(GAIN_ONE); // ±4.096V range
}

void Airflow::calibrateAirflow() {
  Serial.println("Calibrating airflow sensor... Please ensure NO FLOW.");
  delay(2000);

  long sum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t raw = ads.readADC_SingleEnded(0);
    float voltage = (raw * 4.096) / 32767.0;
    sum += voltage * 1000; // store in mV to improve precision
    delay(10);
  }

  this->offsetVoltage = (sum / (float)samples) / 1000.0;

  Serial.print("Calibration complete. Offset voltage = ");
  Serial.print(this->offsetVoltage, 4);
  Serial.println(" V");
}

float Airflow::getAirflow() {
  // Step 1: Read sensor data
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = (raw * 4.096) / 32767.0;

  // Step 2: Apply calibration offset
  voltage -= this->offsetVoltage;

  // Step 3: Convert voltage to differential pressure
  // Example for MPXV7002DP: (Vout - 0.5) * 4.0 = kPa
  float pressure_kPa = (voltage - 0.5) * 4.0;
  this->deltaPressure = pressure_kPa * 1000.0; // Convert to Pa

  // Step 4: Apply Bernoulli’s equation to compute flow
  const float DISCHARGE_COEFFICIENT = 0.98;
  float inletArea = M_PI * pow((inletDiameter / 2.0), 2);
  float throatArea = M_PI * pow((throatDiameter / 2.0), 2);

  this->airFlow = DISCHARGE_COEFFICIENT * throatArea *
                  sqrt((2.0 * deltaPressure) /
                       (airDensity * (1.0 - pow((throatArea / inletArea), 2))));

  return this->airFlow;
}
