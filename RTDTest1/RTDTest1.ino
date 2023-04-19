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
Adafruit_MAX31865 sensor = Adafruit_MAX31865(10, 11, 12, 13);
unsigned long lastRead = 0UL;
unsigned long startTime = millis();

//PID SETUP
double setpoint, input, output; //define pid variables

//pin setup
#define pwmPin 31
#define contrastPin 2

double kp = 190; //proportional gain
double ki = 1.2; //integral gain
double kd = 15; //derivative gain

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT); //setup PID
long now = 0; //initialize current time variable
String status = "hello"; 

LiquidCrystal lcd(3,4,5,6,7,8); //LCD Pin Initialization

void setup()
{
  Serial.begin(9600);
  sensor.begin(MAX31865_3WIRE); //using 3-wire RTD sensor
  
  myPID.SetMode(AUTOMATIC);
  float desiredtemp = 31.5; //desired temperature in Celcius 
  setpoint = mapf(desiredtemp, -50, 280, 0 ,255); //convert temperature to be between 0 and 255 using limits of sensor
  pinMode(pwmPin, OUTPUT);

  lcd.begin(16,2);
  analogWrite(contrastPin, 110);
}
void loop() {
  float desiredtemp = 31.5; //desired temperature in Celcius 
  if (millis() - now > 200){
    now = millis();
    uint16_t rtd_value = sensor.readRTD();
    float ratio = (float)rtd_value / 32768.0f;
    //Serial.print("RTD Resistance: ");
    //Serial.println(RREF * ratio, 8); //print resistance
    //Serial.print("Temperature:  ");
    //Serial.println(sensor.temperature(RNOMINAL, RREF)); //print actual temp value

    double temp = sensor.temperature(RNOMINAL, RREF);
    input = mapf(temp, -50, 280, 0, 255); //map temp value to value read to PWM pin
    double error = abs(setpoint - input);
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    
    Serial.print(temp);
    Serial.print(",");
    Serial.print(desiredtemp);
    Serial.print(",");
    Serial.print(output);
    //Serial.print(",");
    //Serial.print(input);
    Serial.print(",");
    Serial.println(elapsedTime);
    //Serial.print(",");
   //Serial.println(millis());
    lcd.setCursor(0,1);
    lcd.print(temp);
    

    // if (error < 2) {
    //   myPID.SetTunings(10, 0, 0);
    //   status = "cons";
    // }
    // else{ 
    //   myPID.SetTunings(kp, ki, kd);
    //   status ="agg";
    // }
    //Serial.println(status);
    myPID.Compute();
    //int pwmsignal = map(output, -50, 280, 0, 255);
    analogWrite(pwmPin, output);
  }
}
//map function for decimals 
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }