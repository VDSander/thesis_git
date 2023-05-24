from marvelmind import MarvelmindHedge
import math


def getCoordinates(hedges, xCoord, yCoord, index):
    """
    Coordinates of both hedges are placed in an array of N elements that acts as a buffer

    hedges: mobile beacons
    xCoord: array with each element containing the x-coordinates of hedges
    yCoord: array with each element containing the y-coordinates of hedges
    index: variable to achieve circular buffer
    """
    #xCoord[index, :] = [hedgeL.position()[1], hedgeR.position()[1]]
    #yCoord[index, :] = [hedgeL.position()[2], hedgeR.position()[2]]
    if (index == xCoord.shape[0]):
        index = 0
    x1 = hedges[0].position()[1]; x2 = hedges[1].position()[1]
    y1 = hedges[0].position()[2]; y2 = hedges[1].position()[2]
    distanceBetweenPoints = math.sqrt((x1-x2)**2+(y1-y2)**2)
    if distanceBetweenPoints <= 0.25:
        xCoord[index, :] = [x1, x2]
        yCoord[index, :] = [y1, y2]
        index += 1
        
    else: print("Measured coordinates invalid, rejected. Distance: {}".format(distanceBetweenPoints))
    return xCoord, yCoord, index


def getPositionDirection(xCoord, yCoord):
    """
    Calculate actual position and orientation from buffers

    xCoord: buffer of x-coordinates
    yCoord: buffer of y-coordinates

    x: x-position of center robot
    y: y-position of center robot
    direction: angle with respect to x-axis
    """
    x=0; y=0; direction=0
    xs = sum(xCoord)/xCoord.shape[0]; ys = sum(yCoord)/yCoord.shape[0]
    x = sum(xs)/2; y = sum(ys)/2
    
    if(xs[0]!=xs[1]):
        slope = (ys[1]-ys[0])/(xs[1]-xs[0])
    else:
        slope = math.inf*(ys[1]-ys[0])
    if xs[1]>=xs[0]:
        direction = 90 + math.degrees(math.atan(slope)) # Voorlopig in graden, intuitiever
    else: direction = -90 + math.degrees(math.atan(slope))
    #direction = math.pi/4 - math.atan(slope)
    print("Position of robot: \n\tx={}, \n\ty={}".format(x, y))
    return x, y, direction


def getAlfa(x, y, xg, yg, direction):
    """
    Calculate angle to turn to to reach destination

    x: x-position of center robot
    y: y-position of center robot
    xg: x-position of goal
    yg: y-position of goal
    direction: current heading direction

    alfa: required adjustment in direction to reach goal.
    """
    if x!=xg:
        slope = (yg-y)/(xg-x)
    else:
        slope = math.inf*(yg-y)
    angle = math.degrees(math.atan(slope)) # Voorlopig in graden, intuitiever
    if xg<x:
        angle += 180
    
    #angle = math.atan(slope)

    if abs(angle)>90 and abs(direction)>90 and angle*direction<0:
        print("SPECIAL CASE")
        if direction > 0:
            alfa = 360 - (direction + abs(angle))
        else:
            alfa = -(360 - (abs(direction) + angle))

    else:
        alfa = angle-direction # turn right if negative, turn left if positive

    #if abs(alfa) > 180:
    #    alfa = -(alfa%180)
    
    if abs(alfa) > 180:
        alfa = -(360 - abs(alfa)) * alfa/abs(alfa)

    print("\ncurrent direction: {}, angle to goal: {}".format(direction, angle))
    return alfa


def getDistanceToTarget(x, y, xg, yg):
    """
    Calculate distance from center of robot to target destination

    x: x-position of center robot
    y: y-position of center robot
    xg: x-position of goal
    yg: y-position of goal

    distance: distance to target destination 
    """
    distance = ((xg-x)**2 + (yg-y)**2)**0.5
    return distance


