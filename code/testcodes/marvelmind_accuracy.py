from marvelmind import MarvelmindHedge
from time import sleep
import sys
import time
import numpy as np


def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) 
    
    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start()

    N = 100
    coordinates = np.zeros(shape=(N,4), dtype=float)
    index = 0
    offset = 0

    while True:
        try:
            if (hedge.positionUpdated):
                if index < N:
                    coordinates[index,0 + offset] = hedge.position()[1] ; coordinates[index,1 + offset] = hedge.position()[2]
                    index += 1
                elif index == N and offset == 0:
                    print("first measurements finished")
                    print(10)
                    time.sleep(1)
                    print(9)
                    time.sleep(1)
                    print(8)
                    time.sleep(1)
                    print(7)
                    time.sleep(1)
                    print(6)
                    time.sleep(1)
                    print(5)
                    time.sleep(1)
                    print(4)
                    time.sleep(1)
                    print(3)
                    time.sleep(1)
                    print(2)
                    time.sleep(1)
                    print(1)
                    time.sleep(1)
                    index = 0
                    offset = 2
                elif index == N and offset == 2:
                    print("Done measuring")
                    np.savetxt("beacon2_x1.csv", coordinates, delimiter=",")
                    index += 1
                    hedge.stop()
                    sys.exit()

                if index > 0 and index % 10 == 0: print("index: {}".format(index))

            
        except KeyboardInterrupt:
            hedge.stop()
            sys.exit()
main()
