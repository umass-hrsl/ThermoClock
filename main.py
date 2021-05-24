import csv
import datetime
import TempController
import TimeEvent
import EventHandler




def writeToSD(data, fileID):  #data is list of dicts with temp sensor/time and readout
    f = open("log" + fileID, "a")    
    writer = csv.DictWriter(f, fieldnames=['digitalTouch', 'infraredObj', 'infraredAmb', "time"])
    writer.writeheader()
    for i in data:
        writer.writerow(i)
    f.close()
    

if __name__ == '__main__':

    tempData = []
    fileID = datetime.datetime.now().date()

    tempControl = TempController(7, 8)
    eventControl = EventHandler()
    eventControl.addTime(TimeEvent(datetime.datetime.now + datetime.timedelta(hours = 1), 0, 36))
    eventControl.addTime(TimeEvent(datetime.datetime.now + datetime.timedelta(hours = 2), 0, 38.5))

    while True:
        tempData.append(tempControl.control(tempControl))

        if len(tempData) > 10000:
            writeToSD(tempData, fileID)
            tempData.clear()
