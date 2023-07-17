#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <SD.h>

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

//PIN SETUP
#define pwmPin 31
#define contrastPin 2 //lcd contrast

double kp = 190; //proportional gain
double ki = 1.2; //integral gain
double kd = 15; //derivative gain

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT); //setup PID
long now = 0; //initialize current time variable
String status = "hello";

//File Object Initialization
File dataFile;

void setup()
{
  Serial.begin(9600);
  sensor.begin(MAX31865_3WIRE); //using 3-wire RTD sensor
  
  myPID.SetMode(AUTOMATIC);
  setpoint = mapf(36, -50, 280, 0 ,255); //convert temperature to be between 0 and 255 using limits of sensor
  pinMode(pwmPin, OUTPUT);

  Serial.print("Initializing SD card...");
  pinMode(4, OUTPUT);

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  dataFile = SD.open("day4.txt", FILE_WRITE);
}
void loop() {
  if (dataFile) {
    for (int i = 0; i <= 2304000; i++) {
    now = millis();
    uint16_t rtd = sensor.readRTD();
    float ratio = rtd;
    ratio /= 32768;

    double temp = sensor.temperature(RNOMINAL, RREF);
    input = mapf(temp, -50, 280, 0, 255); //map temp value to value read to PWM pin
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    myPID.Compute();
    //int pwmsignal = map(output, -50, 280, 0, 255);
    analogWrite(pwmPin, output);    
    if (i%400 == 0){
      dataFile.print(temp); 
      dataFile.print(",");
      dataFile.println(now);
      Serial.println("one minute passed");
      Serial.println(now);
      Serial.println(i);
      Serial.println(output);
     }
    if (i == 288000){
      setpoint = mapf(38.5, -50, 280, 0, 255);
    }

    if (i == 576000){
      setpoint = mapf(36, -50, 280, 0, 255);
    }

    if (i == 864000){
      setpoint = mapf(38.5, -50, 280, 0, 255);
    }

    if (i == 1152000){
      setpoint = mapf(36, -50, 280, 0, 255);
    }

    if (i == 1440000){
      setpoint = mapf(38.5, -50, 280, 0, 255);
    }

    if (i == 1728000){
      setpoint = mapf(36, -50, 280, 0, 255);
    }

    if (i == 2016000){
      setpoint = mapf(38.5, -50, 280, 0, 255);
    }

    Serial.print(temp);
    Serial.print(",");
    Serial.println(millis());

    }
    dataFile.close();
    Serial.println("protocol done");
  }
}
//map function for decimals 
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }