#include <Arduino.h>
#include "Airflow.h"
#include "Buffer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include "DFRobot_OxygenSensor.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "Wire.h"
#include <math.h>
#include "ads_reader.h"
#include "calculate.h"

#define SDA_PIN 19
#define SCL_PIN 20
const uint8_t O2_ADDR = 0x73;
#define AIRFLOW_ADDR 0x48
#define INLET_SIZE 0.037
#define THROAT_SIZE 0.015   
#define AIRFLOW_BUFFER_SIZE 200
#define AIRVOLUME_BUFFER_SIZE 200
#define CALC_CALORIE_PERIOD_MS 60000
#define ATMOSPHERE_O2 20.9


DFRobot_OxygenSensor oxygen;
SCD30 co2sensor;
Airflow airflow(INLET_SIZE, THROAT_SIZE, AIR_DENSITY);
RingBuffer<double> airflowBuffer(AIRFLOW_BUFFER_SIZE);
RingBuffer<double> airvolumeBuffer(AIRVOLUME_BUFFER_SIZE);
RingBuffer<double> o2Buffer(AIRVOLUME_BUFFER_SIZE);
TimerHandle_t xCalorieTimer = NULL;

using namespace std;

// fase turun (eksalasi / O2 turun) -> hasil 2 titik
const float slope_down_base = 0.964f;
const float offset_down_base = 1.225f;

// fase naik (recovery / balik ke ambient) -> hasil global
const float slope_up_base = 1.0546f;
const float offset_up_base = -1.0998f;

// ====== Parameter pembacaan ======
const uint16_t AVG_N = 20;              // averaging internal library
const uint32_t SAMPLE_MS = 200;         // interval baca ~5 Hz
const float DEAD_BAND = 0.002f;         // deadband arah (%Vol)
const uint32_t FRC_DURATION = 120000UL; // 2 menit fase FRC (ms)

// ====== State & objek ======
DFRobot_OxygenSensor oxygen;

enum TrendMode : uint8_t
{
    MODE_UNKNOWN = 0,
    MODE_DOWN,
    MODE_UP
};
enum Phase : uint8_t
{
    PHASE_FRC = 0,
    PHASE_DYNAMIC
};

TrendMode currentMode = MODE_UNKNOWN;
Phase currentPhase = PHASE_FRC;

float lastRaw = NAN;

// baseline (hasil rata-rata fase FRC)
float baseRaw = NAN;  // rata-rata raw
float baseTrue = NAN; // rata-rata corrected (FRC)

// offset efektif (setelah di-anchoring ke baseline)
float slope_down = slope_down_base;
float offset_down = offset_down_base;
float slope_up = slope_up_base;
float offset_up = offset_up_base;

// akumulasi untuk rata-rata FRC
double sumRawFRC = 0.0;
double sumTrueFRC = 0.0;
uint32_t countFRC = 0;

uint32_t startMillis = 0;

// ====== Helper ======

// ===================== Geometry (SI) =====================
static constexpr float INLET_DIAMETER_M = 0.0370f;  // 3.7 cm
static constexpr float THROAT_DIAMETER_M = 0.0150f; // 1.5 cm
static constexpr float AIR_DENSITY = 1.225f;        // kg/m^3

// ===================== I2C / ADS =====================

ADSReader ads(0x48, GAIN_TWO, SDA_PIN, SCL_PIN); // A0-A1 diff

// ===================== Button (ZERO + RESET) =====================
static constexpr int CAL_BUTTON_PIN = 18; // active-LOW
static constexpr uint32_t DEBOUNCE_MS = 50;

// ===================== MPX10DP Scale =====================
// Can be set at runtime with "K <Pa>"
static float SCALE_PA_PER_VOLT = 4000.0f; // Pa/V (placeholder)

// ===================== Filters & thresholds =====================
static constexpr float LPF_ALPHA = 0.2f;
static constexpr int MA_WINDOW = 5;
static constexpr float INACTIVITY_S = 1.0f;

// Fix zero-creep:
static constexpr float DP_DEADBAND_PA = 0.1f;    // set ΔP=0 if |ΔP|<5 Pa
static constexpr float FLOW_IGNORE_ML_S = 20.0f; // ignore <100 mL/s for integration/breath

// ===================== Calc object =====================
Calculate calc(INLET_DIAMETER_M, THROAT_DIAMETER_M, AIR_DENSITY);

// ===================== State =====================
float zeroVolt = 0.0f;
float totalVolume_L = 0.0f;
float last_Q_L_s = 0.0f;
uint32_t lastMs = 0;

float maBuf[MA_WINDOW];
int maIdx = 0;
bool maFull = false;

struct Sample
{
    float flow_mL_s;
    uint32_t ms;
};
static const size_t FLOW_BUF_MAX = 1024;
Sample breathBuf[FLOW_BUF_MAX];
size_t breathN = 0;
uint32_t lastActiveTick = 0;

float peak_mL_s_5s = 0.0f, sum_mL_s_5s = 0.0f;
int cnt_5s = 0;


double calculateVolume(const RingBuffer<double> &buffer, double samplingRate, size_t samplingSize);
void calibrateSensors(float knownO2Percent = 20.9, float knownCO2ppm = 400.0);
float applyMA(float x);
void clearSession();
void doZero();
void applySpanFromPa(float known_Pa);
void printHelp();
void printParams();
void integrateBreathIfIdle(uint32_t nowTick);
void handleSerial();
static inline float applyCalDown(float raw)
{
    return slope_down * raw + offset_down;
}

static inline float applyCalUp(float raw)
{
    return slope_up * raw + offset_up;
}

// pilih mode naik/turun berdasarkan perubahan raw + deadband
static inline TrendMode decideMode(float rawNow, float rawPrev, TrendMode prevMode)
{
    if (isnan(rawPrev))
        return MODE_DOWN; // default awal
    if (rawNow < rawPrev - DEAD_BAND)
        return MODE_DOWN;
    if (rawNow > rawPrev + DEAD_BAND)
        return MODE_UP;
    return prevMode; // dalam deadband -> jangan gonta-ganti mode
}


void readPressure(void *pvParameters) {
  float airFlow;
  for (;;) {
    float vdiff = ads.readDiffVoltFiltered() - zeroVolt;

   
    float dP_Pa = vdiff * SCALE_PA_PER_VOLT;
    if (fabsf(dP_Pa) < DP_DEADBAND_PA)
        dP_Pa = 0.0f;
    if (dP_Pa < 0)
        dP_Pa = 0;

    
    float Q_mL_s = calc.airflow_mLs(dP_Pa, 0.0f);
    if (Q_mL_s < FLOW_IGNORE_ML_S)
        Q_mL_s = 0.0f; // ignore tiny flows for integration
    float Q_mL_s_view = applyMA(Q_mL_s);
    airflowBuffer.push(Q_mL_s_view);
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void getAirvolume(void *pvParameters) {
  for (;;) {
    double airVolume = calculateVolume(airflowBuffer, 0.01, 50); 
    airvolumeBuffer.push(airVolume);
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

float get02(){
      // 1) Baca raw dari sensor
    float rawNow = oxygen.getOxygenData(AVG_N);

    uint32_t now = millis();

    if (currentPhase == PHASE_FRC)
    {
        // ========= PHASE FRC (2 MENIT PERTAMA) =========
        // pakai garis DOWN base sebagai koreksi
        float corrDown = slope_down_base * rawNow + offset_down_base;

        // akumulasi untuk rata-rata baseline
        sumRawFRC += rawNow;
        sumTrueFRC += corrDown;
        countFRC++;

        // print pakai label FRC (biar keliatan di serial)
        Serial.printf("%.3f\tFRC\t%.3f\n", rawNow, corrDown);

        // kalau sudah lewat 2 menit dan punya cukup data -> kunci baseline
        if (now - startMillis >= FRC_DURATION && countFRC > 10)
        {
            baseRaw = (float)(sumRawFRC / (double)countFRC);
            baseTrue = (float)(sumTrueFRC / (double)countFRC);

            // re-anchoring: paksa kedua garis (DOWN & UP) lewat titik (baseRaw, baseTrue)
            slope_down = slope_down_base;
            slope_up = slope_up_base;
            offset_down = baseTrue - slope_down * baseRaw;
            offset_up = baseTrue - slope_up * baseRaw;

            Serial.println("\n== FRC PHASE DONE ==");
            Serial.printf("Samples FRC : %lu\n", (unsigned long)countFRC);
            Serial.printf("baseRaw     : %.6f %%Vol (rata-rata raw)\n", baseRaw);
            Serial.printf("baseTrue    : %.6f %%Vol (rata-rata corrected/FRC)\n", baseTrue);
            Serial.println("Garis kalibrasi di-ANCHOR ke baseline ini:");
            Serial.printf("  DOWN: corrected = %.6f * raw + %.6f\n", slope_down, offset_down);
            Serial.printf("  UP  : corrected = %.6f * raw + %.6f\n", slope_up, offset_up);
            Serial.println("Keduanya melewati (baseRaw, baseTrue). Masuk PHASE_DYNAMIC.\n");

            currentPhase = PHASE_DYNAMIC;
            currentMode = MODE_DOWN; // mulai dari DOWN saja
            lastRaw = rawNow;        // inisialisasi untuk deteksi arah
        } return lastRaw;
    }
    else
    {
        // ========= PHASE DYNAMIC (setelah FRC) =========

        // 1) tentukan mode naik/turun
        currentMode = decideMode(rawNow, lastRaw, currentMode);

        // 2) apply kalibrasi sesuai mode, tapi SUDAH di-anchor ke baseline
        float corrected;
        const char *modeStr;
        if (currentMode == MODE_UP)
        {
            corrected = applyCalUp(rawNow);
            modeStr = "UP";
        }
        else
        { // MODE_DOWN & MODE_UNKNOWN -> pakai DOWN
            corrected = applyCalDown(rawNow);
            modeStr = (currentMode == MODE_DOWN) ? "DOWN" : "INIT(DOWN)";
        }

        Serial.printf("%.3f\t%s\t%.3f\n", rawNow, modeStr, corrected);

        lastRaw = rawNow;
        return corrected;
    } 

}

void calculateCalorie(TimerHandle_t xTimer) {
  double kCal;
  float co2 = co2sensor.getCO2();
  float o2 = get02();  
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
        Serial.println(" Unknown command.");
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // small delay to yield CPU
  }
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(2000);
    startMillis = millis();
    currentPhase = PHASE_FRC;
  airflow.airflowSetup(SDA_PIN, SCL_PIN, AIRFLOW_ADDR);
  airflow.calibrateAirflow();
  Serial.println("Airflow Calibration Finished");
  ads.setClock(100000);
    ads.setLPFAlpha(LPF_ALPHA);
  if (!ads.beginAuto())
    {
        Serial.println(F("[ERR] ADS1115 not found. Cek wiring & ADDR (0x48..0x4B)."));
        ads.i2cScan();
        while (1)
            delay(300);
    }

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

// ===================== Utils =====================
float applyMA(float x)
{
    maBuf[maIdx] = x;
    maIdx = (maIdx + 1) % MA_WINDOW;
    if (maIdx == 0)
        maFull = true;
    float s = 0;
    int n = maFull ? MA_WINDOW : maIdx;
    if (n <= 0)
        return x;
    for (int i = 0; i < n; ++i)
        s += maBuf[i];
    return s / n;
}

void clearSession()
{
    totalVolume_L = 0.0f;
    last_Q_L_s = 0.0f;
    breathN = 0;
    peak_mL_s_5s = 0.0f;
    sum_mL_s_5s = 0.0f;
    cnt_5s = 0;
    Serial.println(F("[RESET] total volume & buffers cleared"));
}

// Zero offset (avg) + RESET totals
void doZero()
{
    Serial.println(F("[ZERO] Mulai... pastikan dua port setara tekanan"));
    float sum = 0.0f;
    const int N = 1000; // increased for steadier zero
    for (int i = 0; i < N; ++i)
    {
        sum += ads.readDiffVoltFiltered();
        delay(5);
    }
    zeroVolt = sum / N;
    Serial.print(F("[ZERO] zeroVolt="));
    Serial.print(zeroVolt, 6);
    Serial.println(F(" V"));
    clearSession();
}

// Span: set SCALE Pa/V using current Vdiff at known Pa
void applySpanFromPa(float known_Pa)
{
    float vdiff = ads.readDiffVoltFiltered() - zeroVolt;
    if (fabsf(vdiff) < 1e-6f)
    {
        Serial.println(F("[SPAN] Vdiff ~0 V. Naikkan ΔP lalu ulangi."));
        return;
    }
    SCALE_PA_PER_VOLT = known_Pa / vdiff;
    Serial.print(F("[SPAN] Set SCALE_PA_PER_VOLT = "));
    Serial.print(SCALE_PA_PER_VOLT, 2);
    Serial.println(F(" Pa/V"));
}

void printHelp()
{
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  ?        : help"));
    Serial.println(F("  Z        : ZERO + reset total volume"));
    Serial.println(F("  K <Pa>   : span set SCALE = Pa / Vdiff_now"));
    Serial.println(F("  R        : reset total volume only"));
    Serial.println(F("  P        : print parameters & constants"));
    Serial.println();
}

void printParams()
{
    Serial.println(F("==== PARAMETER ===="));
    Serial.print(F("Inlet Dia  (m): "));
    Serial.println(INLET_DIAMETER_M, 6);
    Serial.print(F("Throat Dia (m): "));
    Serial.println(THROAT_DIAMETER_M, 6);
    Serial.print(F("Air density  : "));
    Serial.println(AIR_DENSITY, 3);
    Serial.print(F("Scale Pa/V   : "));
    Serial.println(SCALE_PA_PER_VOLT, 2);
    Serial.print(F("K (mL/s)/sqrt(Pa): "));
    Serial.println(calc.k_mLs(), 3);
    Serial.println(F("==================="));
}

void integrateBreathIfIdle(uint32_t nowTick)
{
    float idle_s = (nowTick - lastActiveTick) * portTICK_PERIOD_MS / 1000.0f;
    if (breathN >= 2 && idle_s >= INACTIVITY_S)
    {
        double mL = 0.0;
        for (size_t i = 1; i < breathN; ++i)
        {
            double dt = (breathBuf[i].ms - breathBuf[i - 1].ms) / 1000.0;
            mL += 0.5 * (breathBuf[i - 1].flow_mL_s + breathBuf[i].flow_mL_s) * dt;
        }
        float L = mL / 1000.0f;
        totalVolume_L += L;

        float peakThis = 0.0f;
        for (size_t i = 0; i < breathN; ++i)
            if (breathBuf[i].flow_mL_s > peakThis)
                peakThis = breathBuf[i].flow_mL_s;

        Serial.print(F("[BREATH] Vol="));
        Serial.print(L, 3);
        Serial.print(F(" L, Peak="));
        Serial.print(peakThis, 0);
        Serial.println(F(" mL/s"));
        breathN = 0;
    }
}

void handleSerial()
{
    static String line;
    while (Serial.available())
    {
        char c = (char)Serial.read();
        Serial.print("Received: ");
        Serial.println(c);
        if (c == '\n' || c == '\r')
        {
            if (line.length())
            {
                line.trim();
                if (line.equalsIgnoreCase("?"))
                    printHelp();
                else if (line.equalsIgnoreCase("Z"))
                    doZero();
                else if (line.equalsIgnoreCase("R"))
                {
                    clearSession();
                }
                else if (line.equalsIgnoreCase("P"))
                    printParams();
                else if (line.startsWith("K "))
                {
                    float pa = line.substring(2).toFloat();
                    if (pa > 0)
                        applySpanFromPa(pa);
                    else
                        Serial.println(F("[ERR] Format: K <Pa>, contoh: K 500"));
                }
                else
                {
                    Serial.println(F("[?] Tidak dikenal. Ketik '?' untuk help."));
                }
                line = "";
            }
        }
        else
            line += c;
    }
}
