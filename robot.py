import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import yaml
import pygame
from pygame.locals import *
import time
from mini_bot import Agent

INPUT_FILE = "Inputs/Paperbot1.csv"
PARAMETER_FILE = "PaperbotParameters.yml"
OUTPUT_FILE = "output.csv"


# $ pip install pygame
# code for pygame taken from this tutorial:
# https://coderslegacy.com/python/python-pygame-tutorial/
def loop(robot, init_state):
    outputFile = open("output.csv", "w")
    w = robot.width
    l = robot.length
    xOffset = robot.S[0] - (l // 2)
    yOffset = robot.S[1] - (w // 2)
    START = (robot.S[0], robot.S[1], robot.S[2])

    states = list()
    # pygame.init()
    # screen = pygame.display.set_mode((P["roomWidth"], P["roomHeight"]))
    with open(INPUT_FILE) as csvFile:
        csvReader = csv.reader(csvFile, delimiter=',')
        inputs = list(csvReader)
        STATE_SIZE = 3

        time = np.linspace(0, 5, num=501)
        states = np.zeros((len(inputs), STATE_SIZE))
        displacement = np.zeros((len(inputs), 2))
        angular_displacement = np.zeros((len(inputs), 1))
        angular_velocity = np.zeros((len(inputs), 1))
        wheel_angular_velocity = np.zeros((len(inputs), 2))

        for i in range(len(inputs)):
            # detect quit
            # for event in pygame.event.get():
            #     if event.type == QUIT:
            #         outputFile.close()
            #         pygame.quit()
            #         sys.exit()
            print(inputs[i])
            robot.state_update(inputs[i])
            states[i, :] = robot.S.T
            displacement[i, :] = np.array((robot.S[0] - START[0], robot.S[1] - START[1])).T
            angular_displacement[i, :] = robot.S[2] - START[2]
            angular_velocity[i, :] = robot.get_IMU_velocity()
            wheel_angular_velocity[i, :] = np.array((robot.wl, robot.wr)).T

            #time.sleep(0.1)
            # print(state)

            # get output
            # output = robot.get_observation()
            # outputText = str(round(output[0][0], 3))[:-1] + " "
            # outputText += str(round(output[0][1], 3))[:-1] + " "
            # outputText += str(round(output[1], 3))[:-1] + " "
            # outputText += str(round(output[2][0], 3))[:-1] + " "
            # outputText += str(round(output[2][1], 3)) + "\n"
            # outputFile.write(outputText)
            # print(outputText)

            # draw
            # screen.fill((0, 0, 0))
            # angle = robot.S[2] * 180 / np.pi
            # surf = pygame.Surface((l, w)).convert_alpha()
            # surf.fill((0, 128, 255))
            # x = xOffset + robot.S[0]
            # y = yOffset + robot.S[1]
            # blitRotate(screen, surf, (x, y), (l // 2, w // 2), -angle)
            # pygame.display.update()

        print("State ls is: ", states.shape)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.plot(states[:, 0], states[:, 1])
        plt.xlabel('x mm')
        plt.ylabel('y mm')
        plt.title(INPUT_FILE[7:-4] + "_Simulation")
        plt.grid()
        ax.set_aspect('equal', adjustable='box')
        #plt.show()
        print("ploted")

        fig, axs = plt.subplots(2, 2)
        axs[0, 0].plot(time, displacement[:, 0], label="x")
        axs[0, 0].plot(time, displacement[:, 1], label="y")
        axs[0, 0].set_xlabel('mm')
        axs[0, 0].set_ylabel('time')
        axs[0, 0].set_title(INPUT_FILE[7:-4] + "_Displacement")
        axs[0, 0].grid()

        axs[0, 1].plot(time, angular_displacement)
        axs[0, 1].set_xlabel('degrees')
        axs[0, 1].set_ylabel('time')
        axs[0, 1].set_title(INPUT_FILE[7:-4] + "_Angular_Displacement")
        axs[0, 1].grid()

        axs[1, 0].plot(time, angular_velocity)
        axs[1, 0].set_xlabel('degrees/sec')
        axs[1, 0].set_ylabel('time')
        axs[1, 0].set_title(INPUT_FILE[7:-4] + "_Angular_Velocity")
        axs[1, 0].grid()

        axs[1, 1].plot(time, wheel_angular_velocity[:, 0], label="left")
        axs[1, 1].plot(time, wheel_angular_velocity[:, 1], label="right")
        axs[1, 1].set_xlabel('degrees/sec')
        axs[1, 1].set_ylabel('time')
        axs[1, 1].set_title(INPUT_FILE[7:-4] + "_Wheel_Angular_Displacement")
        axs[1, 1].grid()

        plt.show()

        # convert radians to degrees
        states[:, 2] = np.degrees(states[:, 2])
        output_matrix = states
        output_matrix = np.column_stack((output_matrix, displacement))
        output_matrix = np.column_stack((output_matrix, angular_displacement))
        output_matrix = np.column_stack((output_matrix, angular_velocity))
        output_matrix = np.column_stack((output_matrix, wheel_angular_velocity))
        np.savetxt(OUTPUT_FILE, output_matrix, delimiter=',', fmt='%.4f')
        # x, y, theta, x_disp, y_disp, theta_disp, omega, left_wheel_omega, right_wheel_omega

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
    surf.blit(rotated_image, (origin[0][0], origin[1][0]))


def main():
    """
    Main Loop for the simulation.
    inputFile will be a csv file seperated by spaces where each line will have two integers
    between 0 and 255. These will represent the 2 inputs
    """
    # load parameters
    P = {}
    with open(PARAMETER_FILE) as pFile:
        P = yaml.load(pFile, Loader=yaml.FullLoader)

    init_state = [P["startingX"], P["startingY"], np.radians(P['startingAngle'])]
    robot = Agent(init_state, P['w'], P['l'], P['d'], P['roomWidth'], P['roomHeight'],
                  P['maxrpm'], P['lstddev'], P['astddev'], P['mstddev'])
    loop(robot, init_state)

    #init_state = [5000, 5000, np.pi/2]

    #robot = Agent(init_state=init_state)
    #loop(P, robot, init_state)


if __name__ == "__main__":
    main()