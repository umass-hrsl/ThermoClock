from TimeEvent import TimeEvent

class EventHandler(object):
    def __init__(self):
        self.events = []

    def __str__(self):
        return self.events

    def addTime(self, timeEvent):
        self.events.append(timeEvent)

    #cycles intervals if mode not equals 0, returns current interval's time remaining, if empty then returns 0
    def timeScheduling(self):   
        if len(self.events) is 0:
            return 0
        else:
            curTime = self.events[0].getTime()
            if curTime > TimeEvent.getNow():
                if(curTime.mode == 0):
                    self.events.pop()
                    if len(self.events) is 1:
                        return 0
                else:
                    time = self.events.pop()
                    self.events.append(time)
                
            curTime = self.events[0]
            return (curTime - TimeEvent.getNow()).seconds
            

    def getClosestTemp(self):
        if len(self.events) is 0:
            return 36
        else:
            return self.events[0].temp

    def getSize(self):
        return self.events.itemCount()


        