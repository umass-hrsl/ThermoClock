from smbus2 import SMBus
from mlx90614 import MLX90614
from simple_pid import PID
import os
from time import sleep


import datetime


class TempController(object):

    def __init__(self, tPin, pPin):
        tempPin = tPin
        padPin = pPin
        # wire = new OneWire(tPin);#TODO
        # tempSensor = new DallasTemperature(wire); #TODO
        # controller = new PID(&input, &output, &setPoint, 10, 1, 0, DIRECT);#TODO
        bus = SMBus(1)
        ir = MLX90614(bus, address = 0x5b) #TODO PUt in actual address


        #PID controller, outputs limited to get around integral windup
        controller = PID(10, 1, 0, setpoint=36, sample_time=0.5, output_limits=(130, 255))#, proportional_on_measurement=True)

        samplingVar = datetime.datetime.now()
        


    # void TempController::init() 
    #     Wire.begin();
    #     #Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
    #     while (ir.begin() == false){ # Initialize thermal IR sensor
    #    
    #     #Serial.println("Qwiic IR Thermometer did acknowledge.");
    #     ir.setUnit(TEMP_C);
        
    #     tempSensor.begin();
    #     controller.SetMode(AUTOMATIC);
    #     controller.SetOutputLimits(125, 255);


    def Control(self, hdlr):   
        print("Start Control:")
        
        res = []
        if((datetime.datetime.now() - self.samplingVar).seconds == 1):
            self.samplingVar += datetime.timedelta(seconds = 1)
            
            print("IR - Object:  + ")
            print(self.ir.object()); #*0.92+2.11)
            print("C   ")
            print("Ambient:  + ")
            print(self.ir.ambient())
            print("C")
            res.append(self.ir.object())
            res.append(self.ir.ambient())
            input = self.ir.object(); #*0.92)+2.611
            
            self.ir.requestTemperatures();  #TODO
        

        #Serial.println("controlling");
        setPoint = hdlr.getClosestTemp()
        self.controller.setpoint = setPoint
        #Serial.println("controlling0");
        error = input - setPoint
        print(input)
        if (error > 0.2):
            output = 0
       
        elif (error > -0.2):
            output = 50 - abs(error*15)
        else:
            if (error > -3 and error <= -1):
                print("farther")
                self.controller.tunings = (3.25, 0.025, 0)
                self.controller.output_limits = (90, 140)

            if (error > -1 and error <= -0.2):
                print("closer")
                self.controller.tunings = (2.75, 0.025, 0)
                self.controller.output_limits = (90, 130)
                # controller.Compute()
        output = self.controller(input)    
       
        
        print("Error: ")
        print(error)
        print("Setpoint: ")
        print(setPoint)
        print("Output: ")
        print(output)
        #print(hdlr.getClosestTime())

        #TODO analogWrite(padPin, output)
        #println("  ")
        #TODO digitalWrite(LED_BUILTIN, HIGH)

        res.append(input)
        res.append(datetime.datetime.now())
        
        return res

