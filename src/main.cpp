#include <Arduino.h>
#include "Airflow.h"
#include "Buffer.h"
#include <freertos/timers.h> 
#include "DFRobot_OxygenSensor.h"

DFRobot_OxygenSensor oxygen;

#define sda 19
#define scl 20
#define O2_ADDR  0x73
#define airflowsensoraddress 0x78
#define inletSize 25
#define throatSize 10
#define airDensity 1.225
#define airflowbufferSize 200
#define airvolumebufferSize 200
#define CALC_CALORIE_PERIOD_MS 60000
#define atmosphereo2 20.9

TimerHandle_t xCalorieTimer = NULL; 
RingBuffer<double> airflowBuffer(airflowbufferSize);
RingBuffer<double> airvolumeBuffer(airvolumebufferSize);
Airflow airflow(inletSize,throatSize,airDensity);
double calculateVolume(const RingBuffer<double>&buffer,double samplingRate, size_t samplingSize);
using namespace std;

void readPressure(void *pvParameters){
    float airFlow;
    while(1){
       airFlow = airflow.getAirflow();
       airflowBuffer.push(airFlow);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void getAirvolume(void *pvParameters){
    while(1){
       double airVolume = calculateVolume(airflowBuffer,0.05,20);
       airvolumeBuffer.push(airVolume);
       vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void calculateCalorie(TimerHandle_t xTimer){
        double kCal;
        float co2;
        double airVolume = airvolumeBuffer.getSum(0,60);
        float o2 = oxygen.getOxygenData(20);
        float vo2 = (atmosphereo2-o2)*airVolume;
        float vco2 = co2*airVolume;
        kCal = (3.941*vo2) + (1.106*vco2) ;
    
}

void setup(){
airflow.airflowSetup(sda,scl,airflowsensoraddress);
if (!oxygen.begin(O2_ADDR))
    {
        Serial.println("oxygen sensor init failed");
        while (1)
            delay(1000);
    }
    xCalorieTimer = xTimerCreate("Weir Timer", pdMS_TO_TICKS(CALC_CALORIE_PERIOD_MS), pdTRUE, ( void * ) 0, calculateCalorie);

    if( xCalorieTimer != NULL )
    {
        xTimerStart( xCalorieTimer, 0 ); 
    }
xTaskCreate(readPressure, "Pressure Sensor", 1000, NULL, 1, NULL);
xTaskCreate(getAirvolume, "air volume", 1000, NULL, 1, NULL);
}

void loop(){
    
}

double calculateVolume(const RingBuffer<double>&buffer,double samplingRate, size_t samplingSize){

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

