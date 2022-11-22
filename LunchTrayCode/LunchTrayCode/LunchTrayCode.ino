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
int sensePin1 = A0;
int sensePin2 = A1;
int sensePin3 = A2;
int sensePin4 = A3;
int sensePin5 = A4;
int sensePin6 = A5;
int sensePin7 = A6;
int sensePin8 = A7;

//voltage divider resistor
int divResistor

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

int rs = 22;
int en = 24;
int d4 = 26;
int d5 = 28;
int d6 = 30;
int d7 = 32;

//assign pin values for LCD
LiquidCrystal lcd(22, 24, 26, 28, 30, 32);

PID* controller = new PID(&input, &output, &setPoint, 25, 0.6, 0, DIRECT);
OneWire* wire = new OneWire(tPin);

void setup() {
  Serial.begin(115200);
  delay(4000);
  pinMode(heatPin, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // LED pin as output

  //Wire.begin();

  controller->SetMode(AUTOMATIC);
  controller->SetOutputLimits(75,225);

  lcd.begin(16,2);
  analogWrite(3,0);
  lastStateCLK = digitalRead(CLK);
  attachInterrupt(digitalPinToInterrupt(CLK), readEncoder, CHANGE);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  temps = control();
  temperatureTouch = temps[0];
  lcd.setCursor(0, 0);
  lcd.print("Set:   Cur:");
  lcd.setCursor(0, 1);
  lcd.print(setPoint);
  lcd.setCursor(7, 1);
  lcd.print(temperatureTouch);

}

float* control() {
  if(millis()-now > 1000){
    now = millis();
    //Read value from RTD sensor and convert to temp
    double val = analogRead(A0);
    val = (1023 / reading)  - 1;
    val = divResistor / val;

    //convert to C
    
    
    //tempSensor->requestTemperatures();  
    //input = tempSensor->getTempCByIndex(0)+2;
    //digitalWrite(LED_BUILTIN, HIGH);
    error = input - setPoint;
    Serial.print(input);
    if (error >= 0){
        controller1->SetOutputLimits(30, setPoint*2.3);
    } 
    else {
      if (error >-1 && error < -0.2) {
        //Serial.println("controlling1");
        controller1->SetTunings(8, 1.5, 0);
        controller1->SetOutputLimits(30, 140);
      }
      if (error >= -0.2 && error < 0){
        controller1->SetTunings(20, 4, 0);
        controller1->SetOutputLimits(35, setPoint*2.5+(setPoint-36)*5);
      }
    }
    controller->Compute();
    
    //Serial.print("    Setpoint: ");
    Serial.print("  ");
    Serial.print(setPoint);
    //Serial.print("    Output: ");
    Serial.print("  ");
    Serial.print(output);
    Serial.print("  ");
    Serial.println("");
    analogWrite(7, output);
    //Serial.println("  ");
    digitalWrite(LED_BUILTIN, HIGH);
    }

  //Serial.println("controlling");
  //Serial.println("controlling0");

  res[0] = float(input);
  
  return res;
}

double mapD(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
