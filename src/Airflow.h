#ifndef AIRFLOW_H
#define AIRFLOW_H

class Airflow {
  private:
    float inletDiameter;
    float throatDiameter;
    float airDensity;
    float voltage;
    float deltaPressure;
    float airFlow;
    float offsetVoltage = 0.0;

  public:
    Airflow(float inletSize, float throatSize, float airdensity);
    void airflowSetup(int sda, int scl, int address);
    void calibrateAirflow();
    float getAirflow();
};

#endif
