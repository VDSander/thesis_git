from pycreate2 import Create2
import time
import string

def passiveMode(bot):
    bot.start()

def collisionRecovery(bot):
    """
    After crashing, robot will drive backward a bit, turn 45 degrees toward midline, drive forward and then continue on its mission
    """
    printThings("crsh")
    toggleLED()
    bot.drive_direct(-100,-100)
    time.sleep(0.5) # Niet werken met sleep maar met een "millis" functie
    bot.drive_stop()
    toggleLED()
    printThings("")

def turnLeft(bot):
    bot.drive_direct(100,-100)

def turnRight(bot):
    bot.drive_direct(-100,100)

def moveForward(bot):
    bot.drive_direct(200,200)

def moveBackward(bot):
    bot.drive_direct(-200,-200)

def toggleLED(bot):
    global lightOn
    lightOn = (lightOn+1)%2
    bot.led(lightOn)

def printThings(bot, word):
    char_set = string.printable
    #word = input("Type a four letter word")
    bot.digit_led_ascii(word)

def quitRobot(bot):
    bot.stop()
    exit()
    
