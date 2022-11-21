#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <limits.h>
#include <LiquidCrystal.h>

#define CLK 0
#define DT 1


const int motor = 10; //LED connected to digital pin 10
const int tPin = 14;
const int heatPin = 7;
unsigned long cur = 0;
const int sdPin = 6;
File myFile;

char *nameOfFile;
float *temps;
float temperatureTouch;
float temperatureIR;
float temperatureIRAM;

double input = 0;
double output = 0;
double setPoint = 37;
double temp;
double error = 0;
long now = 0;
float* res = (float*)malloc(3 * sizeof(float));

int pidCalcTimer = 0;
int tempPin;
int padPin;
int padPower;

int counter = 0;
int currentStateCLK;
int lastStateCLK;

int rs = 12;
int en = 11;
int d4 = 6;
int d5 = 8;
int d6 = 9;
int d7 = 10;

PID* controller1 = new PID(&input, &output, &setPoint, 25, 0.6, 0, DIRECT);
OneWire* wire = new OneWire(tPin);
DallasTemperature* tempSensor = new DallasTemperature(wire);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//IRTherm* ir = new IRTherm();

void setup() {
  Serial.begin(115200);
  /*while (!Serial) {
    Serial.println("Waiting for serial");
  }*/
  delay(4000);
  pinMode(heatPin, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // LED pin as output

  Wire.begin();
//  Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
//  while (ir->begin() == false){ // Initialize thermal IR sensor
//  }
//  Serial.println("Qwiic IR Thermometer did acknowledge.");
//  ir->setUnit(TEMP_C);
  
  tempSensor->begin();
  controller1->SetMode(AUTOMATIC);
  controller1->SetOutputLimits(75, 225);

  lcd.begin(16, 2);
  analogWrite(3,0); 
  lastStateCLK = digitalRead(CLK);
  attachInterrupt(digitalPinToInterrupt(CLK), readEncoder, CHANGE);
  //attachInterrupt(1, readEncoder, CHANGE);
}

void loop() {
  //ArduinoCloud.update();
  digitalWrite(LED_BUILTIN, LOW);
  temps = control();
  //Serial.println(counter);
  temperatureTouch = temps[0];
  lcd.setCursor(0, 0);
  lcd.print("Set:   Cur:");
  lcd.setCursor(0, 1);
  lcd.print(setPoint);
  lcd.setCursor(7, 1);
  lcd.print(temperatureTouch);
  //readEncoder();
  //temperatureIR = temps[1];
  //temperatureIRAM = temps[2];
}

void readEncoder(){
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      if(counter < -100){
        counter = -100;
      }
    } else {
      counter ++;
      if(counter > 100){
        counter = 100;
      }
    }
  }
  lastStateCLK = currentStateCLK;
  setPoint = mapD(counter, -100, 100, 27, 47);
}

float* control() {
  //Serial.println("Start Control:");
  
  //}
  if(millis()-now > 1000){
    now = millis();
    //if (ir->read()){
//      Serial.print("IR - Object:  + ");
//      Serial.print(float(ir->object()));//*0.92+2.11);
//      Serial.print("C   ");
//      Serial.print("Ambient:  + ");
//      Serial.print(float(ir->ambient()));
//      Serial.println("C");
      //res[1] = float(ir->object());
      //res[2] = float(ir->ambient());
      //input = (float(ir->object()));//*0.92)+2.611;
    //} 
    tempSensor->requestTemperatures();  
    input = tempSensor->getTempCByIndex(0)+2;
    //digitalWrite(LED_BUILTIN, HIGH);
    error = input - setPoint;
    Serial.print(input);
    if (error >= 0){
        controller1->SetOutputLimits(30, setPoint*2.3);
    } else {
      /*if (error > -5 && error <= -2) {
        controller->SetTunings(12, 2, 0);
        controller->SetOutputLimits(0, 100);
      }
      if (error > -2 && error <= -1) {
        //Serial.println("controlling1");
        controller->SetTunings(3, 1, 0);
        controller->SetOutputLimits(0, 75);
      }*/
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
    controller1->Compute();
    //controller->Compute();
    
    /*Serial.print("    Error: ");
    Serial.println(error);*/
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
