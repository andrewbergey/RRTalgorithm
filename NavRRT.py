import pygame
import numpy
from NavBase import collision, coords
from NavRobotV2 import *
import itertools


# calculates distance between two points
def rho(p, q):
    (px, py), (qx, qy) = p, q
    rho = ((qx-px)**2+(qy-py)**2)**.5
    return rho


# calculates distance between two points taking into account the angle theta
def sigma(q, pos, theta):
    # vector between two points
    v = (q[0]-pos[0], q[1]-pos[1])
    # unit vector of robots orientation
    u = (numpy.cos(theta), numpy.sin(theta))
    # angle between u and v vectors
    qTheta = numpy.arccos(
        numpy.dot(u, v) / (numpy.linalg.norm(u) * numpy.linalg.norm(v)))
    # distance between two points taking into account a penalty distance for the robots angle facing away from destination
    sig = (rho(pos, q) +
           robotRadius*qTheta/(numpy.pi))
    return sig


def visitEqn(q, pos, theta):
    norm = ((q[0]-pos[0])**2 + (q[1]-pos[1])**2)**.5
    dist = norm + norm*(1 - math.cos(theta))
    return dist


def freePoint():
    point = coords(numpy.random.rand()*200, numpy.random.rand()*200)
    while collision((point[0], point[1])) == True:
        point = coords(numpy.random.rand()*200, numpy.random.rand()*200)
    return point


RRTtree = []
path = []
RRTrobot = []


def nearestNeighbor(queryPoint, verticies):
    finalVertex = verticies[0]
    dist = sigma(queryPoint,
                 verticies[0][0], verticies[0][1])
    i = 1
    while i < len(verticies):
        nextDist = sigma(queryPoint,
                         verticies[i][0],  verticies[i][1])
        if dist > nextDist:
            dist = nextDist
            finalVertex = verticies[i]
        i += 1
    return finalVertex


def new_state(queryPoint, finalVertex, goal, surface):
    v = [1, 50, 100]  # in centimeters
    omega = [-1, 0, 1]
    RRTverticies = [finalVertex]
    RRTrobot.append(Robot(
        finalVertex[0][0], finalVertex[0][1], finalVertex[1], 0, 0, robotImg))

    for combination in itertools.product(v, omega):
        robot = Robot(finalVertex[0][0], finalVertex[0][1], finalVertex[1],
                      combination[0], combination[1], robotImg)
        i = 0
        while i < 200:
            robot.move(.001)
            i += 1
        if collision((robot.x, robot.y)) != True and ([robot.x, robot.y], robot.theta) not in RRTtree:
            RRTverticies.append(([robot.x, robot.y], robot.theta))
            RRTrobot.append(robot)  # appends current robot class

    new_vert = nearestNeighbor(queryPoint, RRTverticies)
    RRTtree.append(new_vert)

    # removes elements in robot class array that are not used in RRT tree creation
    for j in range(len(RRTverticies)):
        if new_vert != RRTverticies[j]:
            RRTrobot.pop(len(RRTrobot)-len(RRTverticies)+j)

    pygame.draw.line(surface, (150, 255, 200), finalVertex[0],
                     new_vert[0], 1)

    path.append((finalVertex, new_vert))

    if rho(goal, new_vert[0]) <= robotRadius:
        return "Reached"
    elif rho(queryPoint, new_vert[0]) < rho(queryPoint, finalVertex[0]):
        return "Advanced"
    else:
        return "Trapped"


def extend(verticies, goal, surface):
    queryPoint = freePoint()
    finalVertex = nearestNeighbor(queryPoint,  verticies)
    state = new_state(queryPoint, finalVertex, goal, surface)
    return state


def pathV7():
    goal_path = [path[-1]]
    robot_path = [RRTrobot[-1]]
    for i in range(len(path)):
        for j in range(len(path)):
            if path[j][1] == goal_path[-1][0] and path[j] not in goal_path:
                goal_path.append(path[j])
                robot_path.append(RRTrobot[j])

    goal_path.reverse()
    robot_path.reverse()
    return goal_path, robot_path
