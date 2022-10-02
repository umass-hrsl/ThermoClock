#include <SPI.h>
#include <SD.h>
//#include <WiFiNINA.h>
#include <limits.h>
#include "TempController.h"
#include "lcdManager.h"
#include "arduino_secrets.h"
#include "thingProperties.h"

const int motor = 10; //LED connected to digital pin 10
const int tPin = 14;
const int heatPin = 7;
unsigned long cur = 0;
const int sdPin = 6;
File myFile;

char *nameOfFile;
float *temps;

EventHandler handler = EventHandler();
TempController thermostat = TempController(tPin, heatPin);
lcdManager lcd = lcdManager(75);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println("Waiting for serial");
  }
  delay(4000);
  Serial.println("Lianhua has woken up!");
  pinMode(motor, OUTPUT); //sets the digital pin as output
  pinMode(heatPin, OUTPUT);

  /*initProperties();
    // Connect to Arduino IoT Cloud
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
    ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT, onIoTConnect);*/
  nameOfFile = initializeSD();
  //strcpy(nameOfFile, initializeSD());
//  Serial.println("Getting File");
  Serial.println(nameOfFile);
  delay(5000);
  handler.init();
  thermostat.init();
  handler.setCycleOnOff(1);
  //lcd.init();
  lcd.input(handler);
  pinMode(LED_BUILTIN, OUTPUT); // LED pin as output
}

//TODO:
//After a long time, 115200 baud creates invalid chars in serial print
//IR sensor lags touch by 1.5 degress on average - ir placed above wellplate, camera end slightly submerged? -> touch sensor half out so top half is getting temp from air?
//Make a case for arduino -? condensation problems
//bad upload
//good temp control -> super constant when reaching target, target temp is kinda ow 38.64, otherwise great.



void loop() {
  //ArduinoCloud.update();
  digitalWrite(LED_BUILTIN, LOW);
  temps = thermostat.control(handler);
  temperatureTouch = temps[0];
  temperatureIR = temps[1];
  temperatureIRAM = temps[2];
  logToSD(temperatureTouch, temperatureIR, temperatureIRAM, handler.getClosestTime(), nameOfFile);
  /*if(cur != ULONG_MAX){
    lcd.printSelectedDelay(handler);
    }else{
    lcd.printHome(handler);
    }*/
}

void onIoTConnect() {
  // enable your other i2c devices
  Serial.println(">>> connected to Arduino IoT Cloud");
  //thermostat.init();
}

char* initializeSD() {
  Serial.println("Initializing SD card...");
  while (!SD.begin(SS1)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
  }
  Serial.println("card initialized.");
  Serial.println("Creating log.txt...");

  // see if the card is present and can be initialized:
  char *fileName  = (char*)malloc(20 * sizeof(char));
  strcpy(fileName, "log(1).csv");
  // Check to see if the file exists:
  char newName[20] = "";
  char *temp = (char*)malloc(10 * sizeof(char));
  int num = 0;
  while (SD.exists(fileName)) {
    Serial.print(fileName);
    Serial.println(" exists."); //have it include date in title after rtc is working
    int subIndex = 0;
    for (int i = 0; i < sizeof(fileName) / sizeof(fileName[0]); i++) {
      subIndex = i;
      while (fileName[i] == '(' && fileName[subIndex] != ')') {
        if (subIndex == i) {
          for (int j = 0; j < subIndex; j++) {
            newName[j] = fileName[j];
          }
          newName[subIndex] = '\0';
        }
        if (fileName[subIndex] != '(' && fileName[subIndex] != ')') {
          strncat(temp, &fileName[subIndex], 1);
        }
        subIndex++;
      }
    }
    num = atoi(temp) + 1;
    strcat(newName, "(");
    char holder[] = "";
    sprintf(holder, "%d", num);
    strcat(newName, holder);
    strcat(newName, ").csv");
    strcpy(fileName, newName);
    memset(temp, 0, strlen(temp));
    num = 0;
  }
  Serial.println(fileName);
  myFile = SD.open(fileName, FILE_WRITE);
  myFile.close();
  return fileName;
}

void logToSD(float touch, float iROB, float iRAM, float curTime, char* fileName) {
  //Serial.println(fileName);
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    myFile.print(touch);
    myFile.print(", ");
    myFile.print(iROB);
    myFile.print(", ");
    myFile.print(iRAM);
    myFile.print(", ");
    myFile.println(curTime);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
  myFile.close();
}

void pump() {
  Serial.print("on");
  digitalWrite(motor, HIGH); //on
  delay(5000);
  Serial.print("off");
  digitalWrite(motor, LOW);
}
