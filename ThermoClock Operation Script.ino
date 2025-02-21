//This code is created to operate the ThermoClock temperature regulation system for cell cand tissue culture 
//Authors: Kelly Zhang (kaiyinzhang@umass.edu) and Dominic Locurto(UMass Amherst alum)
//This example was used for the validation test 3 in our method paper: 

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <SD.h>

// Define constants
#define RREF 430.0 // Reference resistor value for RTD sensors (430.0 for PT100 RTD)
#define NUM_PADS 5 // Number of temperature control modules
#define PWM_PINS {A8, A9, A10, A11, A4} // PWM pins controlling heating pads. Update based on your configuration.
#define RNOMINALS {99.8, 100.05, 100.1, 100.1, 100} // Nominal resistance values for each RTD (~100 for PT100 RTD, need to be calibrated)
//PID gains (can be adjusted as needed)
#define KP 180 // PID proportional gain
#define KI 1.5 // PID integral gain
#define KD 8.5 // PID derivative gain

Adafruit_MAX31865 sensors[NUM_PADS] = { //RTD pins, {CS, DI, DO, CLK}. Update based on your configuration.
    Adafruit_MAX31865(10, 11, 12, 13), // RTD sensor 1
    Adafruit_MAX31865(8, 11, 12, 13), // RTD sensor 2
    Adafruit_MAX31865(4, 11, 12, 13), // RTD sensor 3
    Adafruit_MAX31865(2, 11, 12, 13), // RTD sensor 4
    Adafruit_MAX31865(9, 11, 12, 13) // RTD sensor 5
};

int pwmPins[] = PWM_PINS; // PWM pin array for heating pads
double setpoint[NUM_PADS] = {0}; // Desired temperatures for each pad
double input[NUM_PADS] = {0}; // Measured temperatures for each pad
double output[NUM_PADS] = {0}; // PID outputs for each pad
float rnValues[] = RNOMINALS; // Nominal resistance array for RTDs

PID myPIDs[NUM_PADS] = { //Setup PID controller for individual module
    PID(&input[0], &output[0], &setpoint[0], KP, KI, KD, DIRECT),
    PID(&input[1], &output[1], &setpoint[1], KP, KI, KD, DIRECT),
    PID(&input[2], &output[2], &setpoint[2], KP, KI, KD, DIRECT),
    PID(&input[3], &output[3], &setpoint[3], KP, KI, KD, DIRECT),
    PID(&input[4], &output[4], &setpoint[4], KP, KI, KD, DIRECT)
};

unsigned long startTime; // Start time for logging and scheduling
unsigned long lastLogTime=0; //Last log time in SD card
File dataFile; // File object for SD card logging

// Define temperature control schedule 
struct ModuleSchedule { //Define schedule structure
    double setpoints[10]; // Up to 10 desired temperature setpoints per pad
    double hours[10]; // Hours for each desired temperature setpoint change
    int count; // Number of temperature  changes
};

ModuleSchedule schedules[NUM_PADS] = { 
  //{{temperature steps in celsius},{timepoint in hours},total number of temperature changes}}
    {{36, 38.5, 36, 38.5, 36, 38.5, 37}, {0, 10, 20, 30, 40, 50, 60}, 7}, // Pad 1 schedule
    {{36, 38.5, 36, 38.5, 36, 38.5, 37}, {0, 12, 24, 36, 48, 60, 72}, 7}, // Pad 2 schedule
    {{38.5, 36, 38.5, 36, 38.5, 36, 37}, {0, 12, 24, 36, 48, 60, 72}, 7}, // Pad 3 schedule
    {{36, 38.5, 36, 38.5, 36, 38.5, 37}, {0, 12.5, 25, 37.5, 50, 62.5, 75}, 7}, // Pad 4 schedule
    {{37}, {0}, 1} // Pad 5 schedule
};

void setup() {
    Serial.begin(9600); // Start serial communication

    // Initialize sensors and PID controllers
    for (int i = 0; i < NUM_PADS; i++) {
        sensors[i].begin(MAX31865_3WIRE); // Initialize RTD sensor in 3-wire mode
        myPIDs[i].SetMode(AUTOMATIC); // Set PID mode to automatic
        pinMode(pwmPins[i], OUTPUT); // Set PWM pin as output
    }

    // Initialize SD card for logging
    if (!SD.begin(52)) {
        Serial.println("SD card initialization failed!");
        return; // Stop if SD card initialization fails
    }
    dataFile = SD.open("02112025.TXT", FILE_WRITE); // Open data file for writing. Update file name for each experiment, number only. 
    startTime = millis(); // Record the start time
}

void loop() {
    unsigned long currentTime = millis(); // Get current time in milliseconds
    double elapsedTimeHours = (currentTime - startTime) / 3600000.0; // Convert elapsed time to hours

    // Update setpoints based on schedules
for (int pad = 0; pad < NUM_PADS; pad++) {
    double latestSetpoint = setpoint[pad]; // Default to the current setpoint
    for (int i = 0; i < schedules[pad].count; i++) {
        if (elapsedTimeHours >= schedules[pad].hours[i]) {
            latestSetpoint = schedules[pad].setpoints[i]; // Update to the latest matching setpoint
        } else {
            break; // No need to check further since the schedule is in ascending order
        }
    }
    setpoint[pad] = mapf(latestSetpoint, -50, 280, 0, 255); // Map to PWM range
}

    // Read sensors and compute PID outputs
    for (int i = 0; i < NUM_PADS; i++) {
        double temp = sensors[i].temperature(rnValues[i], RREF); // Get temperature reading
        input[i] = mapf(temp, -50, 280, 0, 255); // Store measured temperature
        myPIDs[i].Compute(); // Compute PID output
        analogWrite(pwmPins[i], output[i]); // Apply PID output to PWM pin

        //Print measured temperatures to Serial Monitor
        Serial.print("Module ");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(temp);
        Serial.print(",");
    }
    Serial.print(currentTime/1000); //in seconds 
    Serial.print(" secs");
    Serial.println();
    //delay(500);

    // Log data to SD card every 30 seconds
    if (currentTime - lastLogTime >= 30000) { // 30 seconds=30000 miliseconds
       if (dataFile) {
            // Log all measured temperatures first (one line)
            double temp[NUM_PADS]={0};
            for (int i = 0; i < NUM_PADS; i++) {
                temp[i]=mapf(input[i],0, 255, -50, 280);
                dataFile.print(temp[i]); // Log measured temperature
                if (i < NUM_PADS - 1) dataFile.print(","); // Separate values with commas
            }
            dataFile.print(","); 

            // Log all desired temperatures next (same line)
            double desired[NUM_PADS]={0};
            for (int i = 0; i < NUM_PADS; i++) {
                desired[i]=mapf(setpoint[i],0, 255, -50, 280);
                dataFile.print(desired[i]); // Log desired temperature
                if (i < NUM_PADS - 1) dataFile.print(","); // Separate values with commas
            }
            dataFile.print(",");
            dataFile.print(currentTime);//log time in miliseconds
            dataFile.println();
            dataFile.flush(); // Ensure data is written to SD card
        }
        lastLogTime = currentTime; // Update last log time
    }
}

// Helper function to map floating-point numbers
// Maps a value from one range to another
// Parameters: value to map, input range min/max, output range min/max
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


