#ifndef AIRFLOW.H
#define AIRFLOW.H 

class Airflow{
    private: 
        const float pi = 3.14;
        float inletDiameter;
        float throatDiameter;
        float airDensity;
        float voltage;
        float deltaPressure;
        float airFlow;
        void getData();
        void convertVolttoPressure();
        void bernoulli();
    public: 
        Airflow (float inletSize, float throatSize, float airdensity);
        void printVoltage();
        void printdeltaPressure();
        float getAirflow();
        void airflowSetup(int sda,int scl, int address);
};

#endif