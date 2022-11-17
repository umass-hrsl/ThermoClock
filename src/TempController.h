#include <Arduino.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <SparkFunMLX90614.h>

#ifndef EVENTHANDLER_H
#define EVENTHANDLER_H

#include "EventHandler.h"

#endif

class TempController{
  private:
    double input = 0;
    double output = 0;
    double setPoint = 0;
    double temp;
    double error = 0;
    long now = 0;
    float* res = (float*)malloc(3 * sizeof(float));


    int pidCalcTimer = 0;
    int tempPin;
    int padPin;
    int padPower;
    PID* controller;
    OneWire* wire;
    DallasTemperature* tempSensor;
    IRTherm* ir;

   
    
  public:
    TempController(int tPin, int pPin);
    void init();
    float* control(EventHandler hdl);
    void cycle(EventHandler hdl);
    void plotResponse();
};
