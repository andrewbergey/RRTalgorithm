import math
import pygame
from NavBase import robotRadius, x0, y0, theta, vel, omega


class Robot:
    def __init__(self, x, y, theta, v, omega, robotImg):
        # robot pose
        self.x = x
        self.y = y
        self.velocity = v
        self.theta = theta
        self.omega = omega
        # robot visual
        self.img2 = pygame.transform.scale(
            robotImg, (2*robotRadius, 2*robotRadius))
        self.img3 = pygame.transform.rotate(
            self.img2, (-self.theta*180/math.pi - 90))
        self.newImg = self.img3
        self.rect = self.newImg.get_rect(
            center=(self.x, self.y))

    # changes robot pose based on class input
    def move(self, dt):
        self.theta += self.omega*dt
        self.x += self.velocity*math.cos(self.theta)*dt
        self.y += self.velocity*math.sin(self.theta)*dt
        self.rect.center = (self.x, self.y)

    # draws robot in current position
    def draw(self, map):
        map.blit(self.newImg, self.rect)


robotImg = pygame.image.load(
    r"insert file path for: robotPhoto.png")


def distanceTraveled(dataLine):
    distTraveled = 0
    robotAction = Robot(x0[dataLine], y0[dataLine], theta[dataLine],
                        vel[dataLine], omega[dataLine], robotImg)
    i = 0
    while i < 200:
        robotAction.move(.001)
        i += 1
    distTraveled += vel[dataLine]*.2
    return robotAction.x, robotAction.y, distTraveled
