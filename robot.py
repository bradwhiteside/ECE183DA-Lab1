import sys
import numpy as np
import math
import csv
import yaml
import pygame
from pygame.locals import *
# $ pip install pygame
# code for pygame taken from this tutorial:
# https://coderslegacy.com/python/python-pygame-tutorial/

class Agent:
  def __init__(self, init_state = [0,0,0], w =90, d=50, rw=10000, rl=10000, maxrpm = 130, lstddev = 0.03, astddev = 8, mstddev =1):  
    self.S = init_state
    self.width = w
    self.diameter = d
    self.room_width = rw		# x-direction
    self.room_length = rl		# y-direction
    self.delta_t = 0.1
    self.wl = 0
    self.wr = 0
    self.MAXRPM = maxrpm
    self.lidarStdDev = lstddev # = 3%
    self.accelerometerStdDev = astddev # = 8 mg-rms
    self.magnemometer = mstddev
    self.PWM_std = [0.01, 0.01]

  def state_update(self, PWM_signal):
    '''
    Moving to the next step given the input u
    return : s
    '''
    u = [0, 0]
    k_l = 1
    k_r = 1

    u[0] = k_l * int(PWM_signal[0])
    u[1] = k_r * int(PWM_signal[1])

    if u[0] < 50:
        u[0] = 0

    if u[1] < 50:
        u[1] = 0

    # Given MAXRPM in revs per min, convert to radians/s and scale input
    self.wl = self.MAXRPM * (np.pi / 30) * (u[0] / 255) + np.random.normal(0, self.PWM_std[0])
    self.wr = self.MAXRPM * (np.pi / 30) * (u[1] / 255) + np.random.normal(0, self.PWM_std[1])

    F = np.eye(3, 3)
    B = np.array([[self.diameter / 4 * np.cos(self.S[2]), self.diameter / 4 * np.cos(self.S[2])],
                  [self.diameter / 4 * np.sin(self.S[2]), self.diameter / 4 * np.sin(self.S[2])],
                  [self.diameter / (2 * self.width), -self.diameter / (2 * self.width)]])

    u = np.vstack((self.wl, self.wr))
    return F @ self.S + B @ u * self.delta_t

    def get_lidar(self):
        x_pos = self.S[0]
        y_pos = self.S[1]
        theta_f = self.S[2]
        theta_r = self.S[2] - math.pi/2
        length = self.room_length
        width = self.room_width

        front_point = [-1,-1]
        right_point = [-1,-1]

        # front sensor find coords
        if math.sin(theta_f) == 0:
            front = [[0, y_pos], [width, y_pos]]
        elif math.cos(theta_f) == 0:
            front = [[x_pos, 0], [x_pos, length]]
        else:
            front = [[0, math.tan(theta_f) * (-1 * x_pos) + y_pos],
                    [width, math.tan(theta_f) * (width - x_pos) + y_pos],
                    [(-1 * y_pos) / math.tan(theta_f) + x_pos, 0],
                    [(length - y_pos)/math.tan(theta_f) + x_pos, length]]

        # front sensor find intersection
        for coord in front:
          if (coord[0] >= 0 and coord[0] <= width and
              coord[1] >= 0 and coord[1] <= length and
              ((math.sin(theta_f) * (coord[0] - x_pos)) + (math.cos(theta_f) * (coord[1] - y_pos))) >= 0):
            front_point = coord

        # right sensor find coords
        if math.sin(theta_r) == 0:
            right = [[0, y_pos], [width, y_pos]]
        elif math.cos(theta_r) == 0:
            right = [[x_pos, 0], [x_pos, length]]
        else:
            right = [[0, math.tan(theta_r) * (-1 * x_pos) + y_pos],
                    [width, math.tan(theta_r) * (width - x_pos) + y_pos],
                    [(-1 * y_pos) / math.tan(theta_r) + x_pos, 0],
                    [(length - y_pos)/math.tan(theta_r) + x_pos, length]]

        # right sensor find intersection
        for coord in right:
          if (coord[0] >= 0 and coord[0] <= width and
              coord[1] >= 0 and coord[1] <= length and
              ((math.sin(theta_r) * (coord[0] - x_pos)) + (math.cos(theta_r) * (coord[1] - y_pos))) >= 0):
            right_point = coord

        # get distances of coords
        if front_point == [-1,-1] or right_point == [-1,-1]:
          raise ValueError("lidar: intersection not found!!! \n" + str(front_point) + "\n" + str(right_point))
        front_lidar = math.sqrt((x_pos - front_point[0])**2 + (y_pos - front_point[1])**2)
        right_lidar = math.sqrt((x_pos - right_point[0])**2 + (y_pos - right_point[1])**2)

        return [front_lidar, right_lidar]
  
  def get_IMU_velocity(self):
    omega = ((self.wl - self.wr)/self.width)*(self.diameter/2)
    return omega + np.random.normal(0, self.accelerometerStdDev)

  def get_IMU_position(self):
    return (math.cos(self.S[2]) + np.random.normal(0, self.magnetometerStdDev), \
            math.sin(self.S[2]) + np.random.normal(0, self.magnetometerStdDev) )
  
  def get_observation(self, wall1, wall2):
    #do the sensor output readings here
    L = self.get_lidaar()
    velocity = self.get_IMU_velocity()
    pos = self.get_IMU_position()
    return L, velocity, pos
  
def get_input(PWM_std):
    i_l = np.ones(100)
    i_r = np.ones(100)

  
"""
Main Loop for the simulation.
inputFile will be a csv file seperated by spaces where each line will have two integers
between 0 and 255. These will represent the 2 inputs
"""
def loop(P, robot):
    xOffset = P["startingX"] - P["d"]
    yOffset = P["startingY"] - P["w"]

    pygame.init()
    screen = pygame.display.set_mode((P["roomWidth"], P["roomHeight"]))

    with open("controls.csv") as csvFile:
        csvReader = csv.reader(csvFile, delimiter=' ')

        while True:
            #detect quit
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()

            #read input
            u = next(csvReader)
            print(u)
            robot.state_update(u)
            print(robot.S)

            #draw
            surf = pygame.Surface((P["d"], P["w"])).convert_alpha()
            surf.fill((0, 128, 255))
            rotated_surf = pygame.transform.rotate(surf, robot.S[2] * 180 / np.pi)
            screen.blit(rotated_surf, (xOffset + robot.S[0], yOffset + robot.S[1]))
            pygame.display.update()

def main():
    #load parameters
    P = {}
    with open("parameters.yml") as pFile:
        P = yaml.load(pFile, Loader=yaml.FullLoader)

    robot = Agent()
    loop(P, robot)

if __name__ == "__main__":
  main()