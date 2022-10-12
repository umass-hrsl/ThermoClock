#include"TempController.h"

TempController::TempController(int tPin, int pPin) {
  tempPin = tPin;
  padPin = pPin;
  wire = new OneWire(tPin);
  //pinMode(A5, INPUT);
  tempSensor = new DallasTemperature(wire);
  controller = new PID(&input, &output, &setPoint, 10, 1, 0, DIRECT);
  ir = new IRTherm();
  
  //init();
}

void TempController::init() {
  Wire.begin();
  Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
  while (ir->begin() == false){ // Initialize thermal IR sensor
  }
  Serial.println("Qwiic IR Thermometer did acknowledge.");
  ir->setUnit(TEMP_C);
  
  tempSensor->begin();
  controller->SetMode(AUTOMATIC);
  controller->SetOutputLimits(125, 255);
}

float* TempController::control(EventHandler hdlr) {
  //Serial.println("Start Control:");
  
  //}
  if(now + 1000 <= millis()){
    now += 1000;
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
    input = tempSensor->getTempCByIndex(0);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  //Serial.println("controlling");
  setPoint = hdlr.getClosestTemp();
  //Serial.println("controlling0");
  error = input - setPoint;
  Serial.print(input);
  if (error > 0.2){
    output = 0;
  }
  else if (error > -0.2) {
    output = 50 - abs(error*15);
  } else {
    if (error > -3 && error <= -1) {
      controller->SetTunings(3.25, 0.025, 0);
      controller->SetOutputLimits(90, 140);
    }
    if (error > -1 && error <= -0.2) {
      //Serial.println("controlling1");
      controller->SetTunings(2.75, 0.025, 0);
      controller->SetOutputLimits(90, 130);
    }
    controller->Compute();
  }
  
  /*Serial.print("    Error: ");
  Serial.println(error);*/
  //Serial.print("    Setpoint: ");
  Serial.print(" ");
  Serial.print(setPoint);
  //Serial.print("    Output: ");
  Serial.print("  ");
  Serial.print(output);
  Serial.print("  ");
  Serial.println(hdlr.getClosestTime());
  analogWrite(padPin, output);
  //Serial.println("  ");
  digitalWrite(LED_BUILTIN, HIGH);

  res[0] = float(input);
  
  return res;
}
