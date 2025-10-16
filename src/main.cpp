#include <Arduino.h>
#include "Airflow.h"
#include "Buffer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include "DFRobot_OxygenSensor.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "Wire.h"

#define SDA_PIN 19
#define SCL_PIN 20
#define O2_ADDR 0x73
#define AIRFLOW_ADDR 0x48
#define INLET_SIZE 25.0
#define THROAT_SIZE 10.0
#define AIR_DENSITY 1.225
#define AIRFLOW_BUFFER_SIZE 200
#define AIRVOLUME_BUFFER_SIZE 200
#define CALC_CALORIE_PERIOD_MS 60000
#define ATMOSPHERE_O2 20.9


DFRobot_OxygenSensor oxygen;
SCD30 co2sensor;
Airflow airflow(INLET_SIZE, THROAT_SIZE, AIR_DENSITY);
RingBuffer<double> airflowBuffer(AIRFLOW_BUFFER_SIZE);
RingBuffer<double> airvolumeBuffer(AIRVOLUME_BUFFER_SIZE);
TimerHandle_t xCalorieTimer = NULL;

using namespace std;


double calculateVolume(const RingBuffer<double> &buffer, double samplingRate, size_t samplingSize);
void calibrateSensors(float knownO2Percent = 20.9, float knownCO2ppm = 400.0);


void readPressure(void *pvParameters) {
  float airFlow;
  for (;;) {
    airFlow = airflow.getAirflow();
    airflowBuffer.push(airFlow);
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void getAirvolume(void *pvParameters) {
  for (;;) {
    double airVolume = calculateVolume(airflowBuffer, 0.05, 20); 
    airvolumeBuffer.push(airVolume);
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}


void calculateCalorie(TimerHandle_t xTimer) {
  double kCal;
  float co2 = co2sensor.getCO2();
  float o2 = oxygen.getOxygenData(20);  
  double airVolume = airvolumeBuffer.getSum(0, 60);  
  double vo2 = (ATMOSPHERE_O2 - o2) * airVolume;
  double vco2 = co2 * airVolume;
  kCal = (3.941 * vo2) + (1.106 * vco2);

  Serial.print("Calorie burn: ");
  Serial.print(kCal, 3);
  Serial.println(" kcal");
}

void commandTask(void *pvParameters) {
  static bool timerRunning = false;
  static bool timerPaused = false;

  for (;;) {
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toLowerCase();

      if (command == "start") {
        if (!timerRunning) {
          if (xCalorieTimer != NULL) {
            xTimerStart(xCalorieTimer, 0);
            timerRunning = true;
            timerPaused = false;
            Serial.println("Measurement started.");
          }
        } else if (timerPaused) {
          xTimerStart(xCalorieTimer, 0);
          timerPaused = false;
          Serial.println("Measurement resumed.");
        } else {
          Serial.println("Timer already running.");
        }
      }

      else if (command == "pause") {
        if (timerRunning && !timerPaused) {
          xTimerStop(xCalorieTimer, 0);
          timerPaused = true;
          Serial.println("Measurement paused.");
        } else {
          Serial.println("Timer not running or already paused.");
        }
      }

      else if (command == "stop") {
        if (timerRunning) {
          xTimerStop(xCalorieTimer, 0);
          timerRunning = false;
          timerPaused = false;
          airflowBuffer.clear();
          airvolumeBuffer.clear();
          Serial.println("Measurement stopped and buffers cleared.");
        } else {
          Serial.println("Timer is not running.");
        }
      }

      else if (command == "restart") {
        if (xCalorieTimer != NULL) {
          xTimerStop(xCalorieTimer, 0);
          airflowBuffer.clear();
          airvolumeBuffer.clear();
          xTimerStart(xCalorieTimer, 0);
          timerRunning = true;
          timerPaused = false;
          Serial.println("Measurement restarted and buffers reset.");
        }
      }

      else if (command == "calibrate") {
        Serial.println("Starting full sensor calibration...");
        xTimerStop(xCalorieTimer, 0); 
        timerRunning = false;
        timerPaused = false;
        calibrateSensors();
        Serial.println("Calibration finished. Use 'start' to begin measurements again.");
      }

      else {
        Serial.println("❓ Unknown command.");
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // small delay to yield CPU
  }
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(2000);

  airflow.airflowSetup(SDA_PIN, SCL_PIN, AIRFLOW_ADDR);
  airflow.calibrateAirflow();
  Serial.println("Airflow Calibration Finished");
  
  if (!oxygen.begin(O2_ADDR)) {
    Serial.println("Oxygen sensor init failed");
    while (1) delay(1000);
  }

  if (!co2sensor.begin(Wire, true)) {
    Serial.println("CO2 sensor not detected. Check wiring.");
    while (1);
  }
  //calibrateSensors();
  xCalorieTimer = xTimerCreate("Calorie Timer", pdMS_TO_TICKS(CALC_CALORIE_PERIOD_MS),
                               pdTRUE, (void *)0, calculateCalorie);

//   if (xCalorieTimer != NULL) {
//     xTimerStart(xCalorieTimer, 0);
//   }
  xTaskCreatePinnedToCore(readPressure, "Pressure Sensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(getAirvolume, "Air Volume", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(commandTask, "Command Handler", 4096, NULL, 1, NULL, 1);
}


void loop() {
  
}

double calculateVolume(const RingBuffer<double> &buffer, double samplingRate, size_t samplingSize) {
  size_t N = samplingSize;
  if (N < 2 || samplingRate <= 0.0 || buffer.size() < N) {
    return 0.0;
  }

  double sum_rates = buffer.get(0) + buffer.get(N - 1);
  for (size_t i = 1; i < N - 1; ++i) {
    sum_rates += 2.0 * buffer.get(i);
  }

  double total_volume = (samplingRate / 2.0) * sum_rates; 
  return total_volume;
}

void calibrateSensors(float knownO2Percent, float knownCO2ppm) {
  Serial.println("\n--- Sensor Calibration Start ---");
  delay(2000);


  Serial.println("Calibrating O2 sensor... Expose to clean ambient air.");
  delay(5000);

  const int o2Samples = 50;
  float o2Sum = 0;
  for (int i = 0; i < o2Samples; i++) {
    float o2Reading = oxygen.getOxygenData(20);
    o2Sum += o2Reading;
    delay(100);
  }

  float o2Avg = o2Sum / o2Samples;
  float o2Offset = knownO2Percent - o2Avg;
  Serial.print("O2 calibration offset = ");
  Serial.print(o2Offset, 3);
  Serial.println(" %");

  Serial.println("\nCalibrating CO2 sensor... Place in outdoor air (~400 ppm).");
  delay(8000);

  float co2Sum = 0;
  int co2Samples = 10;
  for (int i = 0; i < co2Samples; i++) {
    if (co2sensor.dataAvailable()) {
      float co2ppm = co2sensor.getCO2();
      co2Sum += co2ppm;
    }
    delay(2000);
  }

  float co2Avg = co2Sum / co2Samples;
  float co2Offset = knownCO2ppm - co2Avg;
  Serial.print("CO2 calibration offset = ");
  Serial.print(co2Offset, 1);
  Serial.println(" ppm");

  Serial.println("\n--- Calibration Complete ---");
}
