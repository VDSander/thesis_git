from pycreate2.create2api import *
from ina219 import INA219, DeviceRangeError

def getSensorValues(bot):
    sensorValues = bot.get_sensors()
    crashed = False; emergencyStop = False
    if (sensorValues.bumps_wheeldrops[0] or sensorValues.bumps_wheeldrops[1]):
        crashed = True
    if (sensorValues.bumps_wheeldrops[2] or sensorValues.bumps_wheeldrops[3]):
        emergencyStop = True
        print("emergencyStop")
    """
    Om batterijniveau te schatten: 
    1. spanning, stroomvraag en temperatuur meten
    2. Aflezen van spanning omzetten naar SOC schatting via een spanning vs. SOC lookup table
        die bekomen werd via testen of gevonden online
    3. batterijmodel gebruiken om interne weerstand te berekenen, gebaseerd op stroomvraag en temperatuur.
        interne weerstand kan gemodelleerd worden als een functie van T, SOC en andere factoren
    4. aanwezige capaciteit berekenen door verbruikte energie af te trekken van initiele capaciteit
        verbruikte energie berekenen door integratie van stroom over tijdsperiode
    5. SOC benadering updaten
    """
    return crashed, emergencyStop

def chargingStatus(bot, sensorValues):
    bot.start()
    print("Charging Status: ", sensorValues[13])
    print("Battery Charge: ", sensorValues[17])
    print("Charger Available: ", sensorValues[24])
    bot.full()

def getPowerValues(ina):
    try:
        bus_voltage = ina.voltage(); current = ina.current()
        power = ina.power(); shunt_voltage = ina.shunt_voltage()
    except DeviceRangeError as e:
        bus_voltage = 0; current = 0; power = 0; shunt_voltage = 0
        print(e)
    return bus_voltage, current, power, shunt_voltage

def getEstimatedCharge(ina, remaining_charge, time_elapsed):
    # schatting maken van battery life of oplaadtijd adhv gemeten powervalues en dP/dt
    bus_voltage, current, power, shunt_voltage = getPowerValues(ina)
    charging = False
    if current > 0: # charging
        remaining_charge += current * time_elapsed / 3600
        charging = True
    elif current < 0: # discharging
        remaining_charge += current * time_elapsed / 3600

    headToCharger = False
    return remaining_charge