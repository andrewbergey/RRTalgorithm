import numpy

# robot radius
robotRadius = 10

# all dimensions are in centimeters
xlim = 200
ylim = 200
windowSize = [xlim, ylim]


# converting coordinates from top left to bottom left
def coords(x, y):
    [x, y] = [x, ylim - y]
    return (x, y)


# iterate through data points from input1 and store in array
inputCoords = []
with open('input1.data') as input1:
    for line in input1:
        x, y = line.split(',')
        coord = (100*float(x), ylim - 100*float(y))
        inputCoords.append(coord)


# storing each part of input2 into its own array
x0 = []
y0 = []
theta = []
vel = []
omega = []
with open('input2.data') as input2:
    for line in input2:
        dataX, dataY, dataTheta, dataV, dataOmega = line.split(',')
        x0.append(100*float(dataX))  # convert from m to cm
        y0.append(100*float(dataY))  # convert from m to cm
        theta.append(float(dataTheta))
        vel.append(100*float(dataV))  # convert from m to cm
        omega.append(float(dataOmega))


# making map
# wall and boundary start and end coordinates
w1s = coords(50, 115)
w1e = coords(170, 115)
w2s = coords(30, 85)
w2e = coords(170, 85)
w3s = coords(170, 0)
w3e = coords(170, 85)
w4s = coords(170, 115)
w4e = coords(170, 170)
lowerBoundS = coords(0, 0)
lowerBoundE = coords(170, 0)
rightBoundS = coords(200, 0)
rightBoundE = coords(200, 200)
topBoundS = coords(200, 200)
topBoundE = coords(0, 200)
leftBoundS = coords(0, 200)
leftBoundE = coords(0, 0)

# array of coordinates
wallArray = [w1s, w1e, w2s, w2e, w3s, w3e, w4s, w4e, lowerBoundS, lowerBoundE,
             rightBoundS, rightBoundE, topBoundS, topBoundE, leftBoundS, leftBoundE]


# intersection between circle/robot and line
def robotIntersection(circleCenter, radius, lineStart, lineEnd):
    (x1, y1), (x2, y2) = lineStart, lineEnd
    (cx, cy) = circleCenter
    # vector from line start to circle center
    v1 = (cx - x1, cy - y1)
    # vector from line start to line end
    v2 = (x2 - x1, y2 - y1)
    m = v2[0]**2 + v2[1]**2
    # projection of v1 onto v2
    p = numpy.dot(v1, v2)/(m)
    # ensures dot product stays within line segment
    if p >= 1:
        p = 1
    elif p <= 0:
        p = 0
    # finds closest point on line segment to robot center
    cp = (x1 + p*v2[0], y1 + p*v2[1])
    # distance from closest point to center of circle
    dist = ((cx - cp[0])**2 + (cy - cp[1])**2)**.5
    if dist <= radius:
        return 1
    else:
        return 0


# check individual positions for wall collisions
def collision(position):
    l = 0
    intersections = []
    while l < len(wallArray):
        intersection = robotIntersection(
            position, robotRadius+1, wallArray[l], wallArray[l+1])
        intersections.append(intersection)
        l = l+2
    sum = 0
    for j in intersections:
        sum = sum + j
    if sum <= 0:
        return False
    else:
        return True


# storing data from input3 into its own array
inputCoords3 = []
with open('input3.data') as input3:
    for line in input3:
        x, y = line.split(',')
        coord = (100*float(x), ylim - 100*float(y))
        if collision(coord) != True:
            inputCoords3.append(coord)
