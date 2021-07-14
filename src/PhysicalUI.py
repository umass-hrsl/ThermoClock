from EventHandler import EventHandler
from TimeEvent import TimeEvent
import RPi.GPIO as IO
from time import sleep


import datetime


class UI(object):

    def __init__(self, eventHandler):
        IO.setwarnings(False)    # do not show any warnings
        IO.setmode (IO.BCM)      #programming the GPIO by BCM pin numbers (like PIN29 as GPIO5)

        IO.setup(27,IO.IN)      # initialize GPIO27 as an input
        IO.setup(4,IO.IN)
        IO.setup(22,IO.IN)
        IO.setup(17,IO.IN)

        IO.setup(00000, IO.IN) # button, insert actual number here

        handler = eventHandler


    def checkPins():

        if (IO.input(00000, IO.IN) == 1): # button, insert actual number here
            print("Enter Mode - Interval or Alarm")
            userIn = input()
            i = 0
            mode = 0
            while(i == 0):   #TODO FIX FOR TIMING MODE
                if(userIn == 'Interval'):
                    mode = 0
                    i = 1
                    print("Input Interval (mins)")
                    interval = input()
                    while(not interval.isnumeric()):
                        print("Input Interval (mins)")  
                        interval = input()

                    handler.addTime(TimeEvent(datetime.datetime.now + datetime.timedelta(minutes = interval), mode, 36))

                elif(userIn == 'Alarm'):
                    mode = 1
                    i = 1
                    print("Input Time (hrs:mins)")
                    interval = input()
                    splitted = interval.split(",")
                    while(not all(isinstance(x, (int, float)) for x in interval)):
                        print("Input Time (hrs:mins)")
                        interval = input()
                        splitted = interval.split(",")

                    handler.addTime(TimeEvent(datetime.datetime.now + datetime.timedelta(minutes = interval), mode, 36))



                else:
                    print("Enter Mode - Interval or Alarm")
                    userIn = input()
            
            
            
       # else:
       #     if (IO.input(27) == 0):  #If GPIO 27 goes low toggle LED on 21pin and print RIGHT
       #     
       #         print ("RIGHT")
       #
       #     if (IO.input(4) == 1):   #If GPIO 4 goes high toggle LED on 20pin and print LEFT
       #         
       #         print ("LEFT")
       #
       #     if (IO.input(22) == 0):  #If GPIO 22 goes low toggle LED on 16pin and print UP
       #     
       #         print ("UP")
       #
       #    if (IO.input(17) == 1):  #If GPIO 17 goes high toggle LED on 12pin and print DOWN
       #         
       #         print ("DOWN") 
            




        

