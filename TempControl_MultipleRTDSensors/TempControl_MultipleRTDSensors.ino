#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

//TEMP SENSING

// The value of the Rref resistor.
// Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The nominal 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0
// The constructor expects the Arduino pin-numbers
// in the following order: CS, DI, DO, CLK
Adafruit_MAX31865 sensor1 = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 sensor2 = Adafruit_MAX31865(14, 11, 12, 13);
unsigned long lastRead = 0UL;
unsigned long startTime = millis();

//PID SETUP
double setpoint, input1, input2, output;  //define pid variables

//pin setup
#define pwmPin1 31
#define pwmPin2 33
#define pwmPin3 35
#define pwmPin4 37
#define pwmPin5 39
#define pwmPin6 41
#define pwmPin7 43
#define pwmPin8 45
#define contrastPin 2

double kp = 190;  //proportional gain
double ki = 1.2;  //integral gain
double kd = 15;   //derivative gain

PID myPID1(&input1, &output, &setpoint, kp, ki, kd, DIRECT);
PID myPID2(&input2, &output, &setpoint, kp, ki, kd, DIRECT);  //setup PID
long now = 0;                                                 //initialize current time variable
String status = "hello";

LiquidCrystal lcd(3, 4, 5, 6, 7, 8);  //LCD Pin Initialization

void setup() {
  Serial.begin(9600);
  sensor1.begin(MAX31865_3WIRE);  //using 3-wire RTD sensor
  sensor2.begin(MAX31865_3WIRE);

  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  float desiredtemp = 31.5;//desired temperature in Celcius
  setpoint = mapf(desiredtemp, -50, 280, 0, 255);  //convert temperature to be between 0 and 255 using limits of sensor
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  lcd.begin(16, 2);
  analogWrite(contrastPin, 110);
}
void loop() {
  float desiredtemp = 31.5;  //desired temperature in Celcius
  if (millis() - now > 200) {
    now = millis();
    uint16_t rtd_value1 = sensor1.readRTD();
    uint16_t rtd_value2 = sensor2.readRTD();
    float ratio1 = (float)rtd_value1 / 32768.0f;
    float ratio2 = (float)rtd_value2 / 32768.0f;
    //Serial.print("RTD Resistance: ");
    //Serial.println(RREF * ratio, 8); //print resistance
    //Serial.print("Temperature:  ");
    //Serial.println(sensor.temperature(RNOMINAL, RREF)); //print actual temp value

    double temp1 = sensor1.temperature(RNOMINAL, RREF);
    double temp2 = sensor2.temperature(RNOMINAL, RREF);
    input1 = mapf(temp1, -50, 280, 0, 255);  //map temp value to value read to PWM pin
    input2 = mapf(temp2, -50, 280, 0, 255);
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    Serial.print(temp1);
    Serial.print(",");
    Serial.print(temp2);
    Serial.print(",");
    Serial.print(desiredtemp);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.println(elapsedTime);

    myPID1.Compute();
    myPID2.Compute();
    analogWrite(pwmPin1, output);
    analogWrite(pwmPin2, output);
  }
}
//map function for decimals
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}