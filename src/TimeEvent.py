import datetime



class TimeEvent(object):
    def __init__(self, datetime, mode, temp):
        self.date = datetime
        self.mode = mode
        self.temp = temp

    def getTemp(self):
        return self.temp
    
    def getTime(self):
        return self.datetime

    def getTimingMode(self):
        return self.mode

    def getNow():
        return datetime.datetime.now()