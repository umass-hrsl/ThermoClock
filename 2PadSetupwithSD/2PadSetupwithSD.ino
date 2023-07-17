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
Adafruit_MAX31865 sensor1 = Adafruit_MAX31865(30, 32, 34, 36);
Adafruit_MAX31865 sensor2 = Adafruit_MAX31865(47, 45, 43, 41);
unsigned long lastRead = 0UL;
unsigned long startTime = millis();

//PID SETUP
double setpoint, input1, output1, input2, output2; //define pid variables

//PIN SETUP
#define pwmPin1 7
#define pwmPin2 8

double kp = 190; //proportional gain
double ki = 1.2; //integral gain
double kd = 15; //derivative gain

PID myPID1(&input1, &output1, &setpoint,kp,ki,kd, DIRECT); //setup PID
PID myPID2(&input2, &output2, &setpoint,kp,ki,kd, DIRECT); //setup PID
long now = 0; //initialize current time variable

//File Object Initialization
File dataFile;

void setup()
{
  Serial.begin(9600);
  sensor1.begin(MAX31865_3WIRE); //using 3-wire RTD sensor
  sensor2.begin(MAX31865_3WIRE); //using 3-wire RTD sensor
  
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  float desiredtemp = 38.5; //desired temperature in Celcius 
  setpoint = mapf(desiredtemp, -50, 280, 0 ,255); //convert temperature to be between 0 and 255 using limits of sensor
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  Serial.print("Initializing SD card...");
  pinMode(4, OUTPUT);

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  dataFile = SD.open("Inc6.txt", FILE_WRITE);
}
void loop() {
  float desiredtemp = 38.5; //desired temperature in Celcius 
  if (dataFile) {
    for (int i = 0; i <= 4000; i++) {
    now = millis();
    uint16_t rtd1 = sensor1.readRTD();
    uint16_t rtd2 = sensor2.readRTD();
    float ratio1 = rtd1;
    float ratio2 = rtd2;
    ratio1 /= 32768;
    ratio2 /= 32768;

    double temp1 = sensor1.temperature(RNOMINAL, RREF);
    double temp2 = sensor2.temperature(RNOMINAL, RREF);
    input1 = mapf(temp1, -50, 280, 0, 255); //map temp value to value read to PWM pin
    input2 = mapf(temp2, -50, 280, 0, 255); //map temp value to value read to PWM pin
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    myPID1.Compute();
    myPID2.Compute();
    analogWrite(pwmPin1, output1);   
    analogWrite(pwmPin2, output2);   
    if (i%400 == 0){
      dataFile.print(temp1); 
      dataFile.print(",");
      dataFile.print(temp2); 
      dataFile.print(",");
      dataFile.println(now);
      //dataFile.print(",");
      //dataFile.println(output);
      Serial.println("one minute passed");
      Serial.println(now);
      Serial.println(i);
      Serial.println(output1);
      Serial.println(output2);
     }
    // if (i == 4000){
    //   myPID.SetMode(MANUAL);
    //   output = 0;
    // }
    // if (i == 8000){
    //   myPID.SetMode(AUTOMATIC);
    // }
    // Serial.print(",");
    Serial.print(temp1);
    Serial.print(",");
    Serial.print(temp2);
    Serial.print(",");
    // Serial.println(output);
    // Serial.print(",");
    // Serial.print(input);
    // Serial.print(",");
    // Serial.print(",");
    Serial.println(millis());
 
    
    

    // if (error < 2) {
    //   myPID.SetTunings(10, 0, 0);
    //   status = "cons";
    // }
    // else{ 
    //   myPID.SetTunings(kp, ki, kd);
    //   status ="agg";
    // }
    //Serial.println(status);
    }
    dataFile.close();
    Serial.println("protocol done");
  }
}
//map function for decimals 
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }