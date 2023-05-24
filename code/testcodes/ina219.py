from ina219 import INA219
from ina219 import DeviceRangeError
import time

SHUNT_OHMS = 0.1


def read(ina):

    print()
    print("Bus Voltage: %.3f V" % ina.voltage())
    try:
        print("Bus Current: %.3f mA" % ina.current())
        print("Power: %.3f mW" % ina.power())
        print("Shunt voltage: %.3f mV" % ina.shunt_voltage())
        print()
    except DeviceRangeError as e:
        # Current out of device range with specified shunt resistor
        print(e)

def getAddedCapacity(ina, time):
    """
    Function to calculate the added capacity in each time interval during charging.
    When the battery is discharging, the current and thus also the added capacity will be negative
    ina: power measuring device
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
        print(e)



if __name__ == "__main__":
    ina = INA219(SHUNT_OHMS)
    ina.configure()

    measureEnergyTime = 0.0
    available_capacity = 3000
    battery_voltage = 14.4

    while True:
        #read(ina)
        if measureEnergyTime == 0.0:
            measureEnergyTime = time.time()
        elif time.time() >= measureEnergyTime + 0.5:
            available_capacity += getAddedCapacity(ina, time.time()- measureEnergyTime)

        if available_capacity <= 1500:
            print("head to charger")

