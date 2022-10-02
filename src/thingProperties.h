//#include <ArduinoIoTCloud.h>
//#include <Arduino_ConnectionHandler.h>


const char THING_ID[] = "51c62301-bf1c-446f-8ac6-8505bc473754";

const char SSID[]     = SECRET_SSID;    // Network SSID (name)
const char PASS[]     = SECRET_PASS;    // Network password (use for WPA, or use as key for WEP)


float temperatureTouch;
float temperatureIR;
float temperatureIRAM;


void initProperties(){

  /*ArduinoCloud.setThingId(THING_ID);
  ArduinoCloud.addProperty(temperatureTouch, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(temperatureIR, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(temperatureIRAM, READ, 1 * SECONDS, NULL);*/
}

//WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
