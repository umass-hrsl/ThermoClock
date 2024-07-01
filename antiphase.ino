#include <Adafruit_MAX31865.h>  

#include <PID_v1.h>  

#include <SD.h>  

 

//Temperature control system with 5 pads 
//particular version to capsure antiphase entrainment and multiple entrainment schedules
 

// Define the RREF value for the RTD Temperature Sensors 

// Use 430.0 for PT100 and 4300.0 for PT1000  

#define RREF 430.0  

// The nominal 0-degrees-C resistance of the sensor  

// 100.0 for PT100, 1000.0 for PT1000  

#define RNOMINAL 100.35 

#define RNOMINAL_2 100.15  

#define RNOMINAL_3 100.15  

#define RNOMINAL_4 100.25  

#define RNOMINAL_5 100.1  

//#define RNOMINAL_6 100  



 

// Define Arduino pin numbers for the RTD Temperature Sensors  

// in the following order: CS, DI, DO, CLK  

 

Adafruit_MAX31865 sensor1 = Adafruit_MAX31865(10, 11, 12, 13); //J3 

Adafruit_MAX31865 sensor2 = Adafruit_MAX31865(8, 11, 12, 13); //J5 

Adafruit_MAX31865 sensor3 = Adafruit_MAX31865(5, 11, 12, 13); //J8 

Adafruit_MAX31865 sensor4 = Adafruit_MAX31865(2, 11, 12, 13); //J10 

Adafruit_MAX31865 sensor5 = Adafruit_MAX31865(9, 11, 12, 13); //J4 



 

unsigned long lastRead = 0UL;  

 

unsigned long startTime = millis();  

 

//PID SETUP  

 

double setpoint1,setpoint2,setpoint3,setpoint4,setpoint5,setpoint6; 

double input1,input2,input3,input4,input5,input6; 

double output1,output2,output3,output4,output5,output6; 

 

 

//Define Arduino pin numbers for the heating pads outputs  

 

#define pwmPin1 A8 //Pad1 

#define pwmPin2 A9 //Pad2 

#define pwmPin3 A10 //Pad3 

#define pwmPin4 A11 //Pad4 

#define pwmPin5 A4 //Pad5 

//#define pwmPin6 A5 //Pad6 


 

 

double kp = 300; //proportional gain  190 for one pad 

 

double ki = 1.7; //integral gain  1.2 for one pad

 

double kd = 9; //derivative gain  15 for one pad 

 

 

PID myPID1(&input1, &output1, &setpoint1,kp,ki,kd, DIRECT); //setup PID  

PID myPID2(&input2, &output2, &setpoint2,kp,ki,kd, DIRECT); //setup PID  

PID myPID3(&input3, &output3, &setpoint3,kp,ki,kd, DIRECT); //setup PID  

PID myPID4(&input4, &output4, &setpoint4,kp,ki,kd, DIRECT); //setup PID  

PID myPID5(&input5, &output5, &setpoint5,kp,ki,kd, DIRECT); //setup PID  

//PID myPID6(&input6, &output6, &setpoint6,kp,ki,kd, DIRECT); //setup PID  



 

 

long now = 0; //initialize current time variable  

 

 

//File Object Initialization  

 

File dataFile;  

 





 

 

void setup()  

 

{  

 

Serial.begin(9600);  

 

sensor1.begin(MAX31865_3WIRE); //using 3-wire RTD sensor  

sensor2.begin(MAX31865_3WIRE);   

sensor3.begin(MAX31865_3WIRE); 

sensor4.begin(MAX31865_3WIRE); 

sensor5.begin(MAX31865_3WIRE);  

//sensor6.begin(MAX31865_3WIRE); 


 

myPID1.SetMode(AUTOMATIC);  

myPID2.SetMode(AUTOMATIC);  

myPID3.SetMode(AUTOMATIC);  

myPID4.SetMode(AUTOMATIC);  

myPID5.SetMode(AUTOMATIC);  

//myPID6.SetMode(AUTOMATIC);  



 

pinMode(pwmPin1, OUTPUT);  

pinMode(pwmPin2, OUTPUT);  

pinMode(pwmPin3, OUTPUT);  

pinMode(pwmPin4, OUTPUT);  

pinMode(pwmPin5, OUTPUT);  

//pinMode(pwmPin6, OUTPUT);  

 

Serial.print("Initializing SD card...");  

 

pinMode(52, OUTPUT);  

 

 

if (!SD.begin(52)) {  

 

Serial.println("initialization failed!");  

 

return;  

 

}  

 

Serial.println("initialization done.");  

 

 

dataFile = SD.open("0628.txt", FILE_WRITE);  

 

}  

 

void loop() {  

 

if (dataFile) {  

 

for (int i = 0; i <= 870000; i++) {  

 

now = millis();  

 

uint16_t rtd1 = sensor1.readRTD();  

uint16_t rtd2 = sensor2.readRTD();  

uint16_t rtd3 = sensor3.readRTD();  

uint16_t rtd4 = sensor4.readRTD();  

uint16_t rtd5 = sensor5.readRTD();  


 

 

float ratio1 = rtd1;  

float ratio2 = rtd2;  

float ratio3 = rtd3;  

float ratio4 = rtd4;  

float ratio5 = rtd5;  



 

ratio1 /= 32768;  

ratio2 /= 32768;  

ratio3 /= 32768;  

ratio4 /= 32768;  

ratio5 /= 32768;  



 
//Convert RTD reading to temperature in degrees celsius
double temp1 = sensor1.temperature(RNOMINAL, RREF);  

double temp2 = sensor2.temperature(RNOMINAL_2, RREF);  

double temp3 = sensor3.temperature(RNOMINAL_3, RREF);  

double temp4 = sensor4.temperature(RNOMINAL_4, RREF);  

double temp5 = sensor5.temperature(RNOMINAL_5, RREF);  

//double temp6 = sensor6.temperature(RNOMINAL_6, RREF);  


 

input1 = mapf(temp1, -50, 280, 0, 255); //map temp value to value read to PWM pin  

input2 = mapf(temp2, -50, 280, 0, 255); //map temp value to value read to PWM pin  

input3 = mapf(temp3, -50, 280, 0, 255); //map temp value to value read to PWM pin  

input4 = mapf(temp4, -50, 280, 0, 255);  

input5 = mapf(temp5, -50, 280, 0, 255); //map temp value to value read to PWM pin  
 

//setpoints for PID 

float desiredtemp1; 

float desiredtemp2; 

float desiredtemp3; 

float desiredtemp4; 

float desiredtemp5; 


setpoint1 = mapf(desiredtemp1, -50, 280, 0 ,255);  

setpoint2 = mapf(desiredtemp2, -50, 280, 0 ,255); 

setpoint3 = mapf(desiredtemp3, -50, 280, 0 ,255);  

setpoint4 = mapf(desiredtemp4, -50, 280, 0 ,255); 

setpoint5 = mapf(desiredtemp5, -50, 280, 0 ,255);  

 

unsigned long currentTime = millis();  

 

unsigned long elapsedTime = currentTime - startTime;  

 

 

myPID1.Compute();  

myPID2.Compute();  

myPID3.Compute();  

myPID4.Compute();  

myPID5.Compute();  

//myPID6.Compute();  


 

//Match heating pad outputs to the corresponding temperature sensor  

analogWrite(pwmPin1, output1);  

analogWrite(pwmPin2, output2);  

analogWrite(pwmPin3, output3);  

analogWrite(pwmPin4, output4);  

analogWrite(pwmPin5, output5);  

//analogWrite(pwmPin6, output6);  



 

if (i%40 == 0){ //this can change depending on how many pad/sensor pair you have running parallel to each other  
dataFile.print(temp1);  
dataFile.print(",");  
dataFile.print(temp2);  
dataFile.print(","); 
dataFile.print(temp3);  
dataFile.print(",");  
dataFile.print(temp4);  
dataFile.print(",");  
dataFile.print(temp5);  
dataFile.print(",");  
dataFile.print(output1);  
dataFile.print(",");  
dataFile.print(output2);  
dataFile.print(","); 
dataFile.print(output3);  
dataFile.print(",");  
dataFile.print(output4);  
dataFile.print(",");  
dataFile.print(output5);  
dataFile.print(",");   
dataFile.println(now);  
Serial.println("half minute passed");  
Serial.println(now);  
Serial.println(i);  

 

 

}  

//define setpoint temperatures and timepoints 

if (i==5){  
desiredtemp1=35; 
desiredtemp2=35; 
desiredtemp3=35; 
desiredtemp4=35; 
desiredtemp5=35; 
}  

if (i==2400){  
desiredtemp1=37; 
desiredtemp2=37; 
desiredtemp3=37; 
desiredtemp4=37; 
desiredtemp5=37; 
}  

if(i==4800){
desiredtemp1=36;
}
if(i==24000){
desiredtemp2=36;
}
if(i==43200){
desiredtemp3=36;
}
if(i==62400){
desiredtemp4=36;
desiredtemp1=38.5;
}
if(i==81600){
desiredtemp5=36;
desiredtemp2=38.5;
}
if(i==100800){
desiredtemp3=38.5;
}
if(i==120000){
desiredtemp4=38.5;
desiredtemp1=36;
}
if(i==139200){
desiredtemp5=38.5;
desiredtemp2=36;
}
if(i==158400){
desiredtemp3=36;
}
if(i==177600){
desiredtemp4=36;
desiredtemp1=38.5;
}
if(i==196800){
desiredtemp5=36;
desiredtemp2=38.5;
}
if(i==216000){
desiredtemp3=38.5;
}
if(i==235200){
desiredtemp4=38.5;
desiredtemp1=36;
}
if(i==254400){
desiredtemp5=38.5;
desiredtemp2=36;
}
if(i==273600){
desiredtemp3=36;
}
if(i==292800){
desiredtemp4=36;
desiredtemp1=38.5;
}
if(i==312000){
desiredtemp5=36;
desiredtemp2=38.5;
}
if(i==331200){
desiredtemp3=38.5;
}
if(i==350400){
desiredtemp4=38.5;
desiredtemp1=37;
}
if(i==369600){
desiredtemp5=38.5;
desiredtemp2=37;
}
if(i==388800){
desiredtemp3=37;
}
if(i==408000){
desiredtemp4=37;
}
if(i==427200){
desiredtemp5=37;
}






Serial.print(temp1);  
Serial.print(",");  
Serial.print(temp2);  
Serial.print(",");  
Serial.print(temp3);  
Serial.print(",");  
Serial.print(temp4);  
Serial.print(",");  
Serial.print(temp5); 
Serial.print(","); 
Serial.println();  
Serial.print(output1);  
Serial.print(",");  
Serial.print(output2);  
Serial.print(",");  
Serial.print(output3);  
Serial.print(",");  
Serial.print(output4);  
Serial.print(",");  
Serial.print(output5); 
Serial.print(",");  



 

 

 

// Serial.println(output);  

 

// Serial.print(",");  

 

// Serial.print(input);  

 

// Serial.print(",");  

 

// Serial.print(",");  

 

Serial.println(millis());  

 

dataFile.flush();  

 

 

// if (error < 2) {  

 

// myPID.SetTunings(10, 0, 0);  

 

// status = "cons";  

 

// }  

 

// else{  

 

// myPID.SetTunings(kp, ki, kd);  

 

// status ="agg";  

 

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

