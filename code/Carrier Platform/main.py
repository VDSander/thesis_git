from marvelmind import MarvelmindHedge
import sys
from pycreate2 import Create2
import numpy as np
from marvelmindFunctions import *
from create2functions import *
from sensorFunctions import *
import time
import paho.mqtt.client as mqtt
import serial.tools.list_ports
import re

zendstationHandledMessage = False
timeSentToZendstation = 0.0
messageToZendstation = ""

messageFromUser = ""

def restart_bot(bot):
    bot.stop()
    time.sleep(3)
    bot = Create2(bot_port)
    bot.start()
    bot.safe()
    bot.full()

def find_device_port(manufacturer, serial_number):
    device_port = None

    # Get a list of available serial ports
    ports = serial.tools.list_ports.comports()

    # Iterate through each port and look for matching devices
    for port in ports:
        if port.manufacturer == manufacturer and port.serial_number == serial_number:
            device_port = port.device
            break

    return device_port


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("sander/#")


def on_message(client, userdata, msg):
    msgTopic = str(msg.topic)
    
    byte_string = msg.payload
    string_value = byte_string.decode('utf-8')
    msgContent = string_value.strip("b'")
    
    handleMessage(msgTopic, msgContent)


def handleMessage(topic, message):
    if topic == "sander/zendstationToBot":
        print("Laadstation sent a message: '{}'".format(message))
        if message == "number was delivered": 
            global zendstationHandledMessage
            zendstationHandledMessage = True
    elif topic == "sander/userToBot":
        global messageFromUser
        messageFromUser = message
        print("User sent a message: '{}'".format(message))


def getAddedCapacity(ina, time):
    """
    Function to calculate the added capacity in each time interval during charging.
    When the battery is discharging, the current and thus also the added capacity will be negative
    ina: power confirmingArrivation device
    time: measurement time interval
    added_capacity: calculated capacity that was added (positive or negative)
    """
    try:
        voltage = ina.voltage(); current = ina.current()
        power = voltage*current
        added_energy = power*time
        added_capacity = added_energy/(battery_voltage*3600)
        return added_capacity
    except DeviceRangeError as e:
        client.publish(topic="sander/botToUser", payload = e)

def getCurrentVoltage(ina):
    try:
        voltage = ina.voltage(); current = ina.current()
        return current, voltage
    except DeviceRangeError as e:
        client.publish(topic="sander/botToUser", payload = e)
    

def measureCurrentVoltage(ina):
    """
    Function to measure current flowing into the battery during charging and the voltage over its terminals
    ina: power confirmingArrivation device
    morePowerNeeded: True or False
    lessPowerNeeded: True or False
    """
    morePowerNeeded = False; lessPowerNeeded = False
    try:
        voltage = ina.voltage(); current = ina.current()
        if voltage < 15.7 or current < 0.28:
            morePowerNeeded = True
        elif voltage > 17.5 or current > 0.35:
            lessPowerNeeded = True
    except DeviceRangeError as e:
        client.publish(topic="sander/botToUser", payload = e)

    return morePowerNeeded, lessPowerNeeded


def extract_coordinates(string):
    pattern = r'(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)'
    matches = re.findall(pattern, string)
    coordinates = np.array([(float(lat), float(lon)) if '.' in lat or '.' in lon else (int(lat), int(lon)) for lat, lon in matches])
    return coordinates


if __name__== "__main__":
    hedges_manufacturer = 'Marvelmind Robotics'
    hedgeL_serial_number = '207734804646'; hedgeL_port = None
    hedgeR_serial_number = '207834884646'; hedgeR_port = None
    hedgeL_port = find_device_port(hedges_manufacturer, hedgeL_serial_number)
    hedgeR_port = find_device_port(hedges_manufacturer, hedgeR_serial_number)

    hedgeL = MarvelmindHedge(tty = hedgeL_port, adr=None, debug=False)
    hedgeR = MarvelmindHedge(tty = hedgeR_port, adr=None, debug=False)

    hedges = (hedgeL, hedgeR)
    hedges[0].start(); hedges[1].start() # start threads

    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot_port = "/dev/ttyUSB0"
    bot = Create2(bot_port)
    bot.start()
    bot.safe()
    bot.full()

    ina = INA219(shunt_ohms=0.1)
    ina.configure()
    #"""  

    if (len(sys.argv)>1):
        hedges[0].tty, hedges[1].tty = sys.argv[1]


    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("192.108.0.79", 1883, 60) # 192.168.0.79 192.168.0.149

    #"""

    N = 5
    xCoordinates = np.zeros(shape=(N,2), dtype=float); yCoordinates = np.zeros(shape=(N,2), dtype=float)
    xPrecision = np.zeros(shape=(N,2), dtype=float); yPrecision = np.zeros(shape=(N,2), dtype=float)
    xPosition=0; yPosition=0; angle=0; alfa=0
    precisionCoordinates = np.zeros(shape=(1,2), dtype=float); precisionAngle = 0; precisionAlfa = 0
    index = 0; counter = 0; precisionIndex = 0
    calibratingMM = True;   changeDriving = True;   crashed = False;    arrived = False
    confirmingArrivation = False;      headToCharger = False;  charging = False;   recovering = False
    driving = False;        emergencyStop = False;  active = True;      precisionManouvering = False
    measuringAlfa = False;  lostBotConnection = False; reconnectingBot = False
    distanceToGoal = 0; drivingSpeed = 100; precisionDistance = 0; restart = False
    readyForCharging = False; needMorePower = False; needLessPower = False; chargingCompleted = False
    readyForMeasurement = False

    #measuringPoints = np.array([[4.83,-1.58],[4.5,-1.8],[3,-1],[6,-2]])
    #measuringPoints = np.array([[7,-2.12617],[4.0055,-1.12617]])
    measuringPoints = np.empty(shape=[0,2])
    measuringPointsGiven = False
    chargingLocation = [6.5,-1]
    time_elapsed = 0.0;     startMeasuring = 0.0;   startPrecision = 0.0;   charge = 0.0
    startChangeDriving = 0.0;   timeToTurn = 0.0;   restartTime = 0.0;      previousCharge = 0
    timeToDrive = 0.0; botToSafeMode = 0.0; timeSincePowerChange = 0.0;     
    accuracy = 0.03; requestedAccuracy = 0.0
    currentFlow = 0.0; voltage = 0.0
    factor = 0; arrivationCounter = np.zeros(2, dtype = int)

    crash_time = 0.0; timeSinceCrash = 0.0
    time.sleep(1)

    readDataTime = 0.0; charge = 0.0; previousCharge = 0.0

    measureEnergyTime = 0.0; available_capacity = 2500; battery_voltage = 14.4

    while True:
        try:
            client.loop_start()

            if messageFromUser != "":
                print("received message from user: {}".format(messageFromUser))
                """
                Mogelijke opties:
                - handmatig zeggen dat bot klaar is om op te laden - OK
                - bot naar laadstation laten rijden - OK
                - meetlocaties toevoegen - OK
                - alle meetlocaties verwijderen - OK
                - meetlocaties weergeven - OK
                - booleans voor navigatie resetten - OK
                - connectie met bot opnieuw - OK
                - batterijcapaciteit weergeven (ook stroom en spanning) - OK
                - nauwkeurigheid voor locatie instellen - OK
                """
                if messageFromUser == "charge bot": 
                    #bot.drive_stop(); bot.start()
                    readyForCharging = True
                    print("bot is ready for charging")
                    client.publish(topic="sander/botToUser", payload = "Bot is ready for charging")


                if messageFromUser == "go to charger":
                    headToCharger = True
                    client.publish(topic="sander/botToUser", payload = "Bot heading to charger")
                    

                elif "add points" in messageFromUser:
                    print("add points")
                    pointsToAdd = extract_coordinates(messageFromUser)
                    if pointsToAdd.shape[0]>0:
                        measuringPoints = np.concatenate((measuringPoints, pointsToAdd), axis=0)
                    client.publish(topic="sander/botToUser", payload = "Points added")


                elif messageFromUser == "delete points":
                    print("delete all points")
                    measuringPointsGiven = False
                    measuringPoints = np.empty(shape = [0,2])
                    client.publish(topic="sander/botToUser", payload = "Points deleted")
                    

                elif messageFromUser == "show points":
                    print("show all measuring points to user")
                    client.publish(topic="sander/botToUser", payload = str(measuringPoints))


                elif messageFromUser == "reset":
                    xCoordinates = np.zeros(shape=(N,2), dtype=float); yCoordinates = np.zeros(shape=(N,2), dtype=float)
                    xPrecision = np.zeros(shape=(N,2), dtype=float); yPrecision = np.zeros(shape=(N,2), dtype=float)
                    xPosition=0; yPosition=0; angle=0; alfa=0
                    precisionCoordinates = np.zeros(shape=(1,2), dtype=float); precisionAngle = 0; precisionAlfa = 0
                    index = 0; counter = 0; precisionIndex = 0
                    calibratingMM = True;   changeDriving = True;   crashed = False;    arrived = False
                    measuring = False;      headToCharger = False;  charging = False;   recovering = False
                    driving = False;        emergencyStop = False;  active = True;      precisionManouvering = False
                    measuringAlfa = False;  lostBotConnection = False; reconnectingBot = False
                    distanceToGoal = 0; drivingSpeed = 100; precisionDistance = 0; restart = False
                    readyForCharging = False; needMorePower = False; needLessPower = False; chargingCompleted = False
                    readyForMeasurement = False
                    time_elapsed = 0.0;     startMeasuring = 0.0;   startPrecision = 0.0;   charge = 0.0
                    startChangeDriving = 0.0;   timeToTurn = 0.0;   restartTime = 0.0;      previousCharge = 0
                    timeToDrive = 0.0; botToSafeMode = 0.0; timeSincePowerChange = 0.0
                    factor = 0; arrivationCounter = np.zeros(2, dtype = int)
                    crash_time = 0.0; timeSinceCrash = 0.0
                    accuracy = 0.03

                    client.publish(topic="sander/botToUser", payload = "Variables were reset")


                elif messageFromUser == "reset bot":
                    client.publish(topic="sander/botToUser", payload = "Reset bot takes 3 seconds")
                    bot.drive_stop()
                    bot.start()
                    bot.power()
                    bot.close()
                    time.sleep(3)
                    bot = Create2(bot_port)
                    bot.start()
                    bot.full()
                    client.publish(topic="sander/botToUser", payload = "Bot was reset")


                elif "set precision" in messageFromUser:
                    print("set precision")
                    match = re.search(r"(?:\d*\.)?\d+", messageFromUser)
                    if match: requestedAccuracy = float(match.group())
                    else: requestedAccuracy = -1

                    if requestedAccuracy >= 0.02:
                        accuracy = requestedAccuracy 
                        client.publish(topic="sander/botToUser", payload = "Custom precision set")
                    else: client.publish(topic="sander/botToUser", payload = "Failed to set precision")


                elif messageFromUser == "check battery":
                    if currentFlow == 0 or voltage == 0:
                        client.publish(topic="sander/botToUser", payload = "No successful measurement yet")
                    else:
                        message = "Current flow to the battery: " + str(currentFlow) 
                        message += " mA\nVoltage over the battery terminals: " + str(voltage)
                        message += " V\nEstimated capacity of the battery: " + str(available_capacity) + " mAh"
                        client.publish(topic="sander/botToUser", payload = message)

                
                else: client.publish(topic="sander/botToUser", payload = "Message not recognized, consult API")
                

                messageFromUser = ""
            
            if not readyForCharging and not charging:
                if time.time() >= readDataTime + 0.01:
                    sensorValues = bot.get_sensors() 
                    readDataTime = time.time()
                    charge = sensorValues.battery_charge
                    if charge < 200 or charge > 3000: charge = previousCharge
                    if(sensorValues.open_interface_mode  < 3):
                        bot.safe(); bot.full()
                if abs(charge-previousCharge) < 100:
                    previousCharge = charge
                    restart = False
                elif previousCharge == 0:
                    previousCharge = charge
                    restart = False
                else:
                    previousCharge = 0.0
                    restart_bot(bot)
                    restart = True
                    previousCharge = 0.0
                    print("Charge: {}, previousCharge: {}".format(charge, previousCharge))
                    # reset driving values
                    startPrecision = 0; timeToDrive = 0
                    timeToTurn = 0; startChangeDriving = 0


            if measureEnergyTime == 0.0:
                measureEnergyTime = time.time()
            elif time.time() >= measureEnergyTime + 0.5:
                available_capacity += getAddedCapacity(ina, time.time()- measureEnergyTime)
                currentFlow, voltage = getCurrentVoltage(ina)
                measureEnergyTime = time.time()
                if available_capacity <= 1500:
                    headToCharger = True
                    client.publish(topic="sander/botToUser", payload = "bot needs recharging")
                    print("Create 2 needs charging, heading to charger")
                
            
            if headToCharger and not readyForMeasurement:
                printThings(bot, "dock")
                if measuringPoints.shape[0] > 0:
                    if not np.array_equal(measuringPoints[0],chargingLocation):
                        measuringPoints = np.insert(measuringPoints, 0, chargingLocation, axis=0)
                else: measuringPoints = np.insert(measuringPoints, 0, chargingLocation, axis=0)
                
                xGoal = measuringPoints[0][0]; yGoal = measuringPoints[0][1]
                accuracy = 0.03
                """
                andere variabelen terug resetten?
                """


            if charging or readyForCharging:
                
                if charging and time.time() == timeSincePowerChange + 5:
                    needMorePower, needLessPower = measureCurrentVoltage(ina)
                    #when charging started less than 5.5 minutes ago, only check voltage -- not anymore


                # add certain delay and when no message after certain time, notify user via MQTT
                if messageToZendstation != "":
                    if zendstationHandledMessage:
                        messageToZendstation = ""; timeSentToZendstation = 0.0
                        #zendstationHandledMessage = False
                    elif timeSentToZendstation == 0.0:
                        timeSentToZendstation = time.time()
                    elif time.time() >= timeSentToZendstation + 3:
                        client.publish(topic="sander/botToZendstation", payload=messageToZendstation)
                        print("no response, message resent")
                        client.publish(topic="sander/botToUser", payload = "no response from zendstation, trying again")
                        timeSentToZendstation = 0.0                    


                elif readyForCharging:
                    if timeSentToZendstation == 0.0 and not zendstationHandledMessage:
                        print("Device is ready to be charged")
                        messageToZendstation = "start transmission"
                        client.publish(topic="sander/botToZendstation", payload=messageToZendstation)
                        print("Request for charging sent")
                    elif zendstationHandledMessage: 
                        timeSincePowerChange = time.time()
                        charging = True; readyForCharging = False
                        zendstationHandledMessage = False                 


                elif charging and needMorePower:
                    if timeSentToZendstation == 0.0 and not zendstationHandledMessage:
                        print("Device needs more power")
                        messageToZendstation = "more power"
                        client.publish(topic="sander/botToZendstation", payload=messageToZendstation)
                        print("Request for more power sent")
                    elif zendstationHandledMessage:
                        timeSincePowerChange = time.time()
                        zendstationHandledMessage = False
                        needMorePower = False

                
                elif charging and needLessPower:
                    if timeSentToZendstation == 0.0 and not zendstationHandledMessage:
                        print("Device needs less power")
                        messageToZendstation = "less power"
                        client.publish(topic="sander/botToZendstation", payload=messageToZendstation)
                        print("Request for less power sent")
                    elif zendstationHandledMessage:
                        timeSincePowerChange = time.time()
                        zendstationHandledMessage = False
                        needLessPower = False

                elif charging and chargingCompleted:
                    if timeSentToZendstation == 0.0 and not zendstationHandledMessage:
                        print("Device is done charging")
                        messageToZendstation = "stop transmission"
                        client.publish(topic="sander/botToZendstation", payload=messageToZendstation)
                        print("Request for stopping transmission sent")
                    elif zendstationHandledMessage: 
                        charging = False; zendstationHandledMessage = False
                        chargingCompleted = False; measuringPointsGiven = False
                        #alle booleans voor navigatie nog resetten

            
                if available_capacity >= 3000:
                    chargingCompleted = True; measuringPoints = measuringPoints[1:]
                

            elif not measuringPointsGiven and measuringPoints.shape[0] > 0:
                measuringPointsGiven = True
                xGoal = measuringPoints[0][0]; yGoal = measuringPoints[0][1]
                bot = Create2(bot_port); bot.start(); bot.full()


            elif readyForMeasurement:
                if startMeasuring == 0.0:
                    print("bot is ready for measurement")
                    startMeasuring = time.time()
                elif time.time() >= startMeasuring + 3:
                    startMeasuring = 0.0; readyForMeasurement = False


            elif not restart and measuringPointsGiven and not readyForMeasurement:
                crashed, emergencyStop, active = getSensorValues(bot)
                if not active: restart_bot(bot)
                if sensorValues.velocity_right != 0 or sensorValues.velocity_right != 0:
                    driving = True
                else: driving = False

                if not headToCharger and not emergencyStop and not crashed and not confirmingArrivation:
                    printThings(bot, "")

                if emergencyStop:
                    bot.drive_stop()
                    printThings(bot, "stop")

                else:
                    if confirmingArrivation:
                        printThings(bot, "meas")
                        if index == len(xCoordinates):
                            print("Verifying arrivation {} of 5".format(arrivationCounter[1]+1))
                            if distanceToGoal <= accuracy:
                                arrivationCounter[:] += 1
                            else: arrivationCounter[1] += 1
                        if arrivationCounter[1] == 5:
                            print("Verification of arrivation is over, {} were succesful.".format(arrivationCounter[0]))
                            if arrivationCounter[0] >= 4: print("Arrived at destination"); arrived = True
                            else: confirmingArrivation = False; arrived = False; crashed = True
                            arrivationCounter[:] = 0
                        elif arrivationCounter[1] > arrivationCounter[0] + 1:
                            confirmingArrivation = False; arrived = False
                            crashed = True; arrivationCounter[:] = 0
                        
                    if measuringAlfa:
                        print("MeasuringAlfa, counter : {}, precision: {}".format(counter, precisionManouvering))
                        bot.drive_stop()
                        
                        """
                        if startconfirmingArrivation == 0.0:
                            startconfirmingArrivation = time.time()
                        elif time.time()-startconfirmingArrivation >= 1:
                            startconfirmingArrivation = 0.0; measuringAlfa = False
                        # """
                        if precisionManouvering and precisionIndex == len(xPrecision):
                            counter += 1
                        elif index == len(xCoordinates):
                            counter += 1

                        if precisionManouvering and counter == 1: # verhogen indien nog niet nauwkeurig genoeg.
                            counter = 0; measuringAlfa = False
                        elif counter == 2:
                            counter = 0; measuringAlfa = False
    

                    if crashed: 
                        recovering = True
                        if crash_time == 0.0:
                            crash_time = time.time()
                            print("crashed")

                    if recovering: 
                        timeSinceCrash = time.time() - crash_time; print("recovering")
                        if timeSinceCrash < 1: bot.drive_direct(-drivingSpeed,-drivingSpeed)
                        elif timeSinceCrash < 1.5: 
                            if (yPosition<-3.685/2 and abs(angle)<90) or (yPosition>-3.685/2 and abs(angle)>90):
                                bot.drive_direct(drivingSpeed,-drivingSpeed)
                            else: bot.drive_direct(-drivingSpeed, drivingSpeed)
                        elif timeSinceCrash < 3: bot.drive_direct(drivingSpeed,drivingSpeed)
                        else: 
                            recovering = False; #changeDriving = True
                            crash_time = 0.0; print("done recovering")
                            startPrecision = 0; timeToDrive = 0
                            timeToTurn = 0; startChangeDriving = 0

                    if hedgeL.positionUpdated:
                        if hedgeR.positionUpdated:
                            preIndex = index
                            xCoordinates, yCoordinates, index = getCoordinates(hedges, xCoordinates, yCoordinates, index)
                            if index == preIndex: printThings(bot, "mvm")
                            if index == len(xCoordinates):
                                xPosition, yPosition, angle= getPositionDirection(xCoordinates, yCoordinates)
                                alfa = getAlfa(xPosition, yPosition, xGoal, yGoal, angle)
                                distanceToGoal = getDistanceToTarget(xPosition, yPosition, xGoal, yGoal)
                                calibratingMM = False
                                print("turn over angle: {}".format(alfa))
                                print("the distance to the goal is: {}".format(distanceToGoal))
                                if distanceToGoal > 0.5: drivingSpeed = 100; arrived = False
                                elif distanceToGoal > 0.25: drivingSpeed = 50; arrived = False
                                elif distanceToGoal > accuracy: drivingSpeed = 20
                                else: bot.drive_stop(); confirmingArrivation = True; drivingSpeed = 20
                            
                            if precisionManouvering:
                                xPrecision, yPrecision, precisionIndex = getCoordinates(hedges, xPrecision, yPrecision, precisionIndex)

                    if(not charging and not confirmingArrivation and not recovering and xPosition != 0.0 and not arrived and not measuringAlfa and not readyForCharging):
                        if(drivingSpeed == 100 and abs(alfa) > 30): changeDriving = True; precisionManouvering = False
                        elif(drivingSpeed == 50 and abs(alfa) > 20): changeDriving = True; precisionManouvering = False
                        elif(drivingSpeed == 20):
                            precisionManouvering = True; changeDriving = False
                        else: bot.drive_direct(drivingSpeed, drivingSpeed)

                        if precisionManouvering:
                            if precisionIndex == len(xPrecision):
                                """
                                Functies kunnen eventueel gecombineerd worden.
                                """
                                if startPrecision == 0.0:
                                    bot.drive_stop()
                                    precisionCoordinates[0,0], precisionCoordinates[0,1], precisionAngle = getPositionDirection(xPrecision, yPrecision)
                                    precisionAlfa = getAlfa(precisionCoordinates[0,0], precisionCoordinates[0,1], xGoal, yGoal, precisionAngle)
                                    precisionDistance = getDistanceToTarget(precisionCoordinates[0,0], precisionCoordinates[0,1], xGoal, yGoal)
                                    """
                                    implement precise manouvering
                                    """
                                    startPrecision = time.time()
                                    if abs(precisionAlfa) >= 30:
                                        timeToTurn = startPrecision + (abs(precisionAlfa)*15)/360 # It takes the robot about XX seconds for a 360 turn at speed 20
                                        factor = precisionAlfa/abs(precisionAlfa)
                                        bot.drive_direct(int(drivingSpeed*factor), int(-drivingSpeed*factor))
                                    else:
                                        timeToTurn = 0.0
                                        timeToDrive = startPrecision + 1000/3*precisionDistance/drivingSpeed # [1000*meter / millimeter]
                                        bot.drive_direct(drivingSpeed, drivingSpeed)
                            
                            
                            if startPrecision != 0:
                                if timeToTurn != 0 and time.time() >= timeToTurn:
                                    timeToTurn = 0.0; #startPrecision = 0.0 
                                    #bot.drive_stop()
                                    startPrecision = time.time(); bot.drive_direct(drivingSpeed, drivingSpeed)
                                    timeToDrive = startPrecision + 1000/3*precisionDistance/drivingSpeed

                                elif timeToDrive != 0 and time.time() >= timeToDrive:
                                    timeToDrive = 0; measuringAlfa = True; startPrecision = 0.0
                                    bot.drive_stop()
                                
                                


                        elif changeDriving and not measuringAlfa:
                            # alfa ophalen, robot draaien in gewenste richting en vervolgens rijden aan snelheid in functie van afstand

                            """
                            bij 100,-100 en -100,100 duurt een hoek van 360 ongeveer 4.55 seconden 
                            """
                            if startChangeDriving == 0.0:
                                startChangeDriving = time.time()
                                if drivingSpeed == 100:
                                    timeToTurn = startChangeDriving + (abs(alfa)*6.7)/360 # It takes the robot about 6.7 seconds for a 360 turn at speed 100
                                elif drivingSpeed == 50:
                                    timeToTurn = startChangeDriving + (abs(alfa)*8)/360 # It takes the robot about XX seconds for a 360 turn at speed 50
                            elif time.time() >= timeToTurn and timeToTurn != 0:
                                changeDriving = False; measuringAlfa = True
                                startChangeDriving = 0.0; timeToTurn = 0.0
                                bot.drive_stop()

                                bot.drive_direct(drivingSpeed, drivingSpeed)
                            elif time.time() < timeToTurn: #and alfa != 0.0:
                                print(alfa)
                                bot.drive_direct(int(drivingSpeed*(alfa/abs(alfa))),int(drivingSpeed*(-alfa/abs(alfa))))

                        else: 
                            bot.drive_direct(drivingSpeed, drivingSpeed)

                        #if not measuringAlfa and timeToTurn == 0.0:
                        #    if timeToDrive == 0: 
                        #        timeToDrive = time.time() + 1000/3*distanceToGoal/drivingSpeed
                        #        bot.drive_direct(drivingSpeed, drivingSpeed)
                        #    elif time.time() >= timeToDrive:
                        #        timeToDrive == 0; bot.drive_stop()

                    if arrived:
                        if headToCharger: 
                            headtoCharger = False; readyForCharging = True
                            confirmingArrivation = False; measuringAlfa = False
                            # power down the bot
                            bot.start(); bot.power(); bot.close()
                            measuringPoints = measuringPoints[1:]
                        elif confirmingArrivation: 
                            print("Destination reached"); confirmingArrivation= False; 
                            measuringAlfa = False; readyForMeasurement = True
                            if measuringPoints.shape[0] > 1:
                                measuringPoints = measuringPoints[1:]
                                xGoal = measuringPoints[0][0]; yGoal = measuringPoints[0][1]
                            else: 
                                print("All tasks fullfilled"); printThings(bot, "Rdy")
                                measuringPoints = np.empty(shape=[0,2]); measuringPointsGiven = False
                                bot.start(); bot.power(); bot.close()
            
        except KeyboardInterrupt:
            bot.drive_stop()
            hedges[0].stop(); hedges[1].stop()  # stop and close serial port
            bot.start(); bot.power(); bot.close()
            sys.exit()
