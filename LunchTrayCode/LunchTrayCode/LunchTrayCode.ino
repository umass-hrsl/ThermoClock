// initialize libraries
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <limits.h>
#include <LiquidCrystal.h>

//define pins for CLK pins on encoder (DT 90 deg away from CLK)
//Might not need these for the new code since we have new resistive heating pads (no data pin) and I want to make setpoint programmable for each board 
#define CLK 0
#define DT 1

//assign pin values for LCD
LiquidCrystal lcd(22, 24, 26, 28, 30, 32);

//pins for Pads (transistor pinout based on diagram)
int padPin1 = 9;
int padPin2 = 8;
int padPin3 = 7;
int padPin4 = 6;
int padPin5 = 5;
int padPin6 = 4;
int padPin7 = 3;
int padPin8 = 2;

//sensor pins
int heaterPin1 = A0;
int heaterPin2 = A1;
int heaterPin3 = A2;
int heaterPin4 = A3;
int heaterPin5 = A4;
int heaterPin6 = A5;
int heaterPin7 = A6;
int heaterPin8 = A7;

//init PID parameters
double input = 0;
double output = 0;
double setPoint = 37;
double temp;
double error = 0;
long now = 0;
float* res = (float*)malloc(3 * sizeof(float));

int counter = 0;
int currentStateCLK;
int lastStateCLK;

int rs = 12;
int en = 11;
int d4 = 6;
int d5 = 8;
int d6 = 9;
int d7 = 10;



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
