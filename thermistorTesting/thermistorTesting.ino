// the value of the 'other' resistor
float resistor = 100000.0;
float Vin = 5.0;
   
 
// What pin to connect the sensor to
#define tempPin A0 
 
void setup() {
  Serial.begin(9600);
  pinMode(tempPin, INPUT);
}
 
void loop() {
  float reading = analogRead(tempPin);
  float voltage = reading * (Vin/1023.0);
  float rt = ((Vin*resistor)/voltage) - resistor;
  Serial.println(voltage,4);
  Serial.println(rt,3);
  delay(500);
}
