import pygame
from NavBase import *
from NavRobotV2 import *
from NavRRT import *
from time import sleep

# colors
white = (255, 255, 255)
black = (0, 0, 0)

# creating screen
screen = pygame.display.set_mode(windowSize)
screen.fill(white)
pygame.display.set_caption('Project 1: Navigation')

# loop drawing walls and boundaries using walls array
k = 0
while k < len(wallArray):
    pygame.draw.line(screen, black, wallArray[k], wallArray[k+1], 1)
    k += 2

# Part 1 check each point from input1 to find if it collides with wall
i = 0
trueSum = 0
falseSum = 0
while i < len(inputCoords)/10:  # shorten loop to speed up
    robotOrigin = inputCoords[i]
    tf = collision(robotOrigin)
    if tf == True:
        trueSum += 1
    else:
        falseSum += 1
    i += 1

# Part 1 find % area not drivable by robot
print("Not drivable percentage of map =", 100*(trueSum)/(falseSum+trueSum))

# part 2 testing each pose from input2
i = 0
distTraveled = 0
while i < len(x0):
    if (collision(coords(x0[i], y0[i])) != True) and (collision(coords(distanceTraveled(i)[0], distanceTraveled(i)[1])) != True):
        distTraveled += distanceTraveled(i)[2]
    i += 1

print("Total Distance Traveled by robot (m) =", distTraveled/100)

# initial conditions for RRT
goal = coords(185.0, 0.0)
startPos = (coords(125.0, 25.0), math.pi/3)
RRTtree.append(startPos)


# draw robot start and goal point
pygame.draw.circle(screen, (0, 255, 0), goal, 5)


# running loop
pygame.display.update()

running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # running RRT algorithm
    state = extend(RRTtree, goal, screen)
    iterations = 0
    while state != "Reached":
        state = extend(RRTtree, goal, screen)
        iterations += 1
        # clears clicks out of queue(prevents freezing when using mouse and keyboard)
        pygame.event.get()
        pygame.display.update()
    print("Finished State:", state)
    print("Total Iterations:", iterations)

    # creates the final path to goal
    goal_path = pathV7()

    # draws final path to goal
    for edge in goal_path[0]:
        pygame.draw.line(screen, (0, 0, 255), edge[0][0], edge[1][0])

    RRTbackground = pygame.Surface(screen.get_size())
    RRTbackground.blit(screen, (0, 0))

    for element in goal_path[1]:
        i = 0
        while i < 200:
            screen.blit(RRTbackground, (0, 0))
            element.move(.001)
            element.draw(screen)

            pygame.display.update()
            pygame.event.get()
            sleep(.001)
            i += 1
        if rho((element.x, element.y), goal) < robotRadius:
            break

    # part 3 iterating through input3 to see which points were visited
    visited = []
    for combination in itertools.product(inputCoords3, RRTtree):
        if visitEqn(combination[0], combination[1][0], combination[1][1]) < robotRadius:
            visited.append(
                (round(combination[0][0], 4), round(combination[0][1], 4)))
    visited = [*set(visited)]  # gets rid of duplicate visited points
    freeSpace = [*set(inputCoords3)]  # gets rid of duplicate input points
    print("Ratio of free space visted:", len(visited)/len(freeSpace))

    pygame.display.update()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
