#include <Adafruit_MAX31865.h> 

#include <PID_v1.h> 

#include <SD.h> 


//Arduino IDE sketch for one-pad Bioclockbot temperature regulation system 
//Author:Kaiyin(Kelly) Zhang (kaiyinzhang@umass.edu) & Dominic Locurto(dlocurto@fas.harvard.edu)


// RTD Setup
 
// The value of the Rref resistor. 

// Use 430.0 for PT100 and 4300.0 for PT1000 

#define RREF 430 

// The nominal 0-degrees-C resistance of the sensor 

// 100.0 for PT100, 1000.0 for PT1000 

#define RNOMINAL 100.15 

// The constructor expects the Arduino pin-numbers for the Adafruit_MAX31865 RTD Amplifier

// in the following order: CS, DI, DO, CLK 

Adafruit_MAX31865 sensor = Adafruit_MAX31865(9, 11, 12, 13); 

unsigned long lastRead = 0UL; 

unsigned long startTime = millis(); 

 

//PID Setup 

double setpoint, input, output; //define pid variables 

double kp = 190; //define proportional gain 

double ki = 1.2; //define integral gain 

double kd = 15; //define derivative gain 

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT); //setup PID 


 
//Heating pad Setup
//The constructor expects the Arduino pin-number output to the bipolar transistor collector end (for the heating pad)

#define pwmPin A8 


long now = 0; //initialize current time variable 

String status = "hello"; 

 

//File Object Initialization 

File dataFile; 

 

void setup() 

{ 

  Serial.begin(9600); 

  sensor.begin(MAX31865_3WIRE); //using 3-wire RTD sensor 

   

  myPID.SetMode(AUTOMATIC); 

  desiredtemp=36;//define initial desired temperature 

  setpoint = mapf(desiredtemp, -50, 280, 0 ,255); //convert temperature to be between 0 and 255 using limits of sensor 
 

  pinMode(pwmPin, OUTPUT); //define pin to the bipolar transistor as output pin

 
  //for the system to start operating, the SD card must be initialized successfully
 
  Serial.print("Initializing SD card..."); 

  pinMode(4, OUTPUT); //define the proper chip selection (CS) pin to the micro-SD breakout. Pin 4 or 52 on Arduino DUE

 

  if (!SD.begin(4)) { 

    Serial.println("initialization failed!"); 

    return; 

  } 

  Serial.println("initialization done."); 

 

  dataFile = SD.open("Binder79.txt", FILE_WRITE); //name the file storing the data for this experiment 

} 

void loop() { 

  dataFile.seek(EOF); //begin data logging at the end of the file 

  if (dataFile) { 

    for (int i = 0; i <= 3000000; i++) { //i <= xxxx would determine the total time lapse of the experiment. In one pad, i=400 is a minute.

    now = millis(); 

    uint16_t rtd = sensor.readRTD(); 

    float ratio = rtd; 

    ratio /= 32768; 
 
    double temp = sensor.temperature(RNOMINAL, RREF);  //Real-time temperature reading from the RTD is converted numerical temperature (temp)

    input = mapf(temp, -50, 280, 0, 255); //map temp value to value read to PWM pin 

    unsigned long currentTime = millis(); 

    unsigned long elapsedTime = currentTime - startTime; 

    myPID.Compute(); 

    //int pwmsignal = map(output, -50, 280, 0, 255); 

    analogWrite(pwmPin, output);     

    if (i%200 == 0){ //every i=400 is a minute, user can change the i%xxx depending on the desired frequency of data logging (default is i%200, log once every half a minute)

     //define information to be logged into the SD card (user can also choose to log the output and input from the PID computation)
      dataFile.print(temp);  

      dataFile.print(","); 

      dataFile.println(now); 

     //define infromation to be printed on the serial monitor 

      Serial.println("half a minute passed"); 

      Serial.println(now); 

      Serial.println(i);   

     } 

 

    if (i == 288000){ //first setpoint changes (12h after start), before this the system would try to hit the setup desired temperature and sustains  

      setpoint = mapf(38.5, -50, 280, 0, 255);  

      Serial.print("daytime"); 

    } 

    if (i == 576000){ //Second setpoint (24h after start)

      setpoint = mapf(36, -50, 280, 0, 255);  

      Serial.print("nighttime"); 

    } 

    if (i == 864000){ //Third setpoint(36h after start)

      setpoint = mapf(38.5, -50, 280, 0, 255);  

      Serial.print("daytime"); 

    } 

    if (i == 1152000){ //Fourth setpoint (48h after start)
      setpoint = mapf(36, -50, 280, 0, 255);  

      Serial.print("nighttime"); 

    } 

    if (i == 1440000){ //Firth setpoint (60h after start)

      setpoint = mapf(38.5, -50, 280, 0, 255);  

      Serial.print("daytime"); 

    }       

 

    if (i == 1728000){ //Sixth setpoint (72h after start)

      setpoint = mapf(37, -50, 280, 0, 255);  

      Serial.print("FreeRun"); 

    } 

//Information displayed on the serial monitor 
     
    Serial.print(temp); 

    Serial.print(","); 

    Serial.print(output); 

    Serial.print(","); 

    Serial.println(millis()); 

    dataFile.flush(); //enable continuous data logging to the SD card) 

 

    } 

    dataFile.close(); 

    Serial.println("protocol done"); 

  } 

} 

//map function for decimals  

double mapf(double val, double in_min, double in_max, double out_min, double out_max) { 

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; } 
