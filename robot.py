import numpy as np
import math
import csv
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
    
  def state_update(self, u):
    '''
    Moving to the next step given the input u
    
    return : s
    '''
    d = self.diameter
    w = self.width
    
    F = np.eye(3,3)
    B = np.array([[d/4 * np.cos(self.theta), d/4 * np.cos(self.theta)], 
                 [d/4 * np.sin(self.theta), d/4 * np.sin(self.theta)],
                 [self.d/(2*self.w),        -self.d/(2*self.w)]])
    
    #Given MAXRPM in revs per min, convert to radians/s and scale input 
    self.wl = self.MAXRPM * (np.pi/30) * (u[0]/255)
    self.wr = self.MAXRPM * (np.pi/30) * (u[1]/255)
    return F @ self.S + B @ self.u * self.delta_t
                 
    
    
  
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
  
  def get_IMU_velocity():
    omega = ((self.wl - self.wr)/self.width)*(self.diameter/2)
    return omega + np.random.normal(0, self.accelerometerStdDev)
      
  def get_IMU_position():
    return (cos(self.S[2]) + np.random.normal(0, self.magnetometerStdDev), \
            sin(self.S[2]) + np.random.normal(0, self.magnetometerStdDev) )
  
  def get_observation(self, wall1, wall2):
    #do the sensor output readings here
    L = self.get_lidaar()
    velocity = self.get_IMU_velocity()
    pos = self.get_IMU_position()
    return L, velocity, pos
    
    
    
  
def get_input(PWM_std):
  i_l = np.ones(100)
  i_r = np.ones(100)
     
	return u
  
"""
Main Loop for the simulation.
inputFile will be a csv file seperated by spaces where each line will have two integers
between 0 and 255. These will represent the 2 inputs
"""
def loop(screen, robot):
  with open("./controls.txt") as csvFile:
    csvReader = csv.reader(csvFile, delimiter=' ')
    
    while True:
    	#detect quit
    	pygame.display.update()
    	for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
      
      #read input
    	u = next(csvReader)
     	robot.state_update(u)
      
  
def main():
  pygame.init()
  with open("./parameters.txt") as parametersFile:
    screen = pygame.display.set_mode((screenWidth, screenHeight))
		pygame.draw.rect(screen, (0,128,255), pygame.Rect(startingX, startingY, d, l)
  	Agent robot = Agent()
    loop(robot)

if __name__ == "__main__":
  main()