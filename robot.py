import sys
import numpy as np
import math
import csv
import yaml
import pygame
from pygame.locals import *
import time


# $ pip install pygame
# code for pygame taken from this tutorial:
# https://coderslegacy.com/python/python-pygame-tutorial/

class Agent:
    def __init__(self, init_state=[0, 0, 0], w=90, d=50, rw=10000, rl=10000, maxrpm=130, lstddev=0.03, astddev=8,
                 mstddev=1):
        self.S = np.reshape(np.array(init_state), (3, 1))
        self.width = w
        self.diameter = d
        self.room_width = rw  # x-direction
        self.room_length = rl  # y-direction
        self.delta_t = 0.1
        self.wl = 0
        self.wr = 0
        self.MAXRPM = maxrpm
        self.lidarStdDev = lstddev  # = 3%
        self.accelerometerStdDev = astddev  # = 8 mg-rms
        self.magnetometerStdDev = mstddev
        self.PWM_std = [0, 0]

    def PWM_to_RPM(self, x):
        s = np.sign(x)
        k = 0.05
        if (np.abs(x) > 100):
            return self.MAXRPM * s

        return self.MAXRPM * s * np.exp(s * k * x) / (np.exp(k * 100))

    def state_update(self, PWM_signal):
        '''
        Moving to the next step given the input u
        return : s
        '''
        u = [0, 0]
        k_l = 1
        k_r = 1

        u[0] = self.PWM_to_RPM(float(PWM_signal[0]))
        u[1] = self.PWM_to_RPM(float(PWM_signal[1]))

        # if u[0] < 50:
        #     u[0] = 0

        # if u[1] < 50:
        #     u[1] = 0
        self.wl = self.MAXRPM * (np.pi / 30) * (u[0] / 100)  # + np.random.normal(0, self.PWM_std[0])
        self.wr = self.MAXRPM * (np.pi / 30) * (u[1] / 100)  # + np.random.normal(0, self.PWM_std[1])
        # v = u[0]
        # ommega = u[1]
        u = np.vstack((self.wl, self.wr))

        B = np.array([[np.cos(self.S[2, 0]), 0],
                      [np.sin(self.S[2, 0]), 0],
                      [0, 1]])

        Dynamixs = np.array([[self.diameter / 4, self.diameter / 4],
                             [-self.diameter / (2 * self.width), self.diameter / (2 * self.width)]])

        self.S = + self.S + (B @ Dynamixs @ u) * self.delta_t
        # self.S[2,0] = pi_2_pi(self.S[2,0])

        return self.S

    def get_lidar(self):
        x_pos = self.S[0]
        y_pos = self.S[1]
        theta_f = self.S[2]
        theta_r = self.S[2] - math.pi / 2
        length = self.room_length
        width = self.room_width

        front_point = [-1, -1]
        right_point = [-1, -1]

        # front sensor find coords
        if math.sin(theta_f) == 0:
            front = [[0, y_pos], [width, y_pos]]
        elif math.cos(theta_f) == 0:
            front = [[x_pos, 0], [x_pos, length]]
        else:
            front = [[0, math.tan(theta_f) * (-1 * x_pos) + y_pos],
                     [width, math.tan(theta_f) * (width - x_pos) + y_pos],
                     [(-1 * y_pos) / math.tan(theta_f) + x_pos, 0],
                     [(length - y_pos) / math.tan(theta_f) + x_pos, length]]

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
                     [(length - y_pos) / math.tan(theta_r) + x_pos, length]]

        # right sensor find intersection
        for coord in right:
            if (coord[0] >= 0 and coord[0] <= width and
                    coord[1] >= 0 and coord[1] <= length and
                    ((math.sin(theta_r) * (coord[0] - x_pos)) + (math.cos(theta_r) * (coord[1] - y_pos))) >= 0):
                right_point = coord

        # get distances of coords
        if front_point == [-1, -1] or right_point == [-1, -1]:
            raise ValueError("lidar: intersection not found!!! \n" + str(front_point) + "\n" + str(right_point))
        front_lidar = math.sqrt((x_pos - front_point[0]) ** 2 + (y_pos - front_point[1]) ** 2)
        right_lidar = math.sqrt((x_pos - right_point[0]) ** 2 + (y_pos - right_point[1]) ** 2)
        return [front_lidar, right_lidar]

    def get_IMU_velocity(self):
        omega = ((self.wl - self.wr) / self.width) * (self.diameter / 2)
        return omega + np.random.normal(0, self.accelerometerStdDev)

    def get_IMU_position(self):
        return (math.cos(self.S[2, 0]) + np.random.normal(0, self.magnetometerStdDev), \
                math.sin(self.S[2, 0]) + np.random.normal(0, self.magnetometerStdDev))

    def get_observation(self):
        # do the sensor output readings here
        L = self.get_lidar()
        velocity = self.get_IMU_velocity()
        pos = self.get_IMU_position()
        return L, velocity, pos


"""
Main Loop for the simulation.
inputFile will be a csv file seperated by spaces where each line will have two integers
between 0 and 255. These will represent the 2 inputs
"""


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def loop(P, robot):
    outputFile = open("output.csv", "w")
    w = P["w"]
    l = P["l"]
    xOffset = P["startingX"] - (l // 2)
    yOffset = P["startingY"] - (w // 2)

    pygame.init()
    screen = pygame.display.set_mode((P["roomWidth"], P["roomHeight"]))
    font = pygame.font.Font('freesansbold.ttf', 32)

    with open("controls.csv") as csvFile:
        csvReader = csv.reader(csvFile, delimiter=' ')

        while True:
            # detect quit
            for event in pygame.event.get():
                if event.type == QUIT:
                    outputFile.close()
                    pygame.quit()
                    sys.exit()

            # read input
            u = next(csvReader)
            robot.state_update(u)
            time.sleep(0.1)

            # get output

            output = robot.get_observation()
            outputText = str(round(output[0][0], 3))[:-1] + " "
            outputText += str(round(output[0][1], 3))[:-1] + " "
            outputText += str(round(output[1], 3))[:-1] + " "
            outputText += str(round(output[2][0], 3))[:-1] + " "
            outputText += str(round(output[2][1], 3)) + "\n"
            outputFile.write(outputText)
            print(outputText)

            # draw
            screen.fill((0, 0, 0))
            angle = robot.S[2] * 180 / np.pi
            surf = pygame.Surface((l, w)).convert_alpha()
            surf.fill((0, 128, 255))
            x = xOffset + robot.S[0]
            y = yOffset + robot.S[1]
            blitRotate(screen, surf, (x, y), (l // 2, w // 2), -angle)

            pygame.display.update()


# adjust coords so the surface rotates about its center
# https://stackoverflow.com/questions/4183208/how-do-i-rotate-an-image-around-its-center-using-pygame
def blitRotate(surf, image, pos, originPos, angle):
    # calcaulate the axis aligned bounding box of the rotated image
    w, h = image.get_size()
    box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
    box_rotate = [p.rotate(angle) for p in box]
    min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
    max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

    # calculate the translation of the pivot
    pivot = pygame.math.Vector2(originPos[0], -originPos[1])
    pivot_rotate = pivot.rotate(angle)
    pivot_move = pivot_rotate - pivot

    # calculate the upper left origin of the rotated image
    origin = (pos[0] - originPos[0] + min_box[0] - pivot_move[0], pos[1] - originPos[1] - max_box[1] + pivot_move[1])

    # get a rotated image
    rotated_image = pygame.transform.rotate(image, angle)

    # rotate and blit the image
    surf.blit(rotated_image, origin)


def main():
    # load parameters
    P = {}
    with open("parameters.yml") as pFile:
        P = yaml.load(pFile, Loader=yaml.FullLoader)

    robot = Agent()
    loop(P, robot)


if __name__ == "__main__":
    main()