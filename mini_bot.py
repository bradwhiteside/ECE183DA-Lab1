import numpy as np
import math
class Agent:
    def __init__(self, init_state=[0, 0, 0], w=530, d=502, rw=10000, rl=10000, maxrpm=130, lstddev=0.03, astddev=8,
                 mstddev=1):
        self.S = np.reshape(np.array(init_state), (3, 1))
        print(self.S)
        self.width = w
        self.diameter = d
        self.room_width = rw  # x-direction
        self.room_length = rl  # y-direction
        self.delta_t = 0.01
        self.wl = 0
        self.wr = 0
        self.MAXRPM = 1/6
        self.lidarStdDev = lstddev  # = 3%
        self.accelerometerStdDev = astddev  # = 8 mg-rms
        self.magnetometerStdDev = mstddev
        self.PWM_std = [0, 0]
        self.Wheel_std = [0, 0]

    def PWM_to_RPM(self, x):
        s = np.sign(x)
        k = 0.05
        if (np.abs(x) > 100):
            return self.MAXRPM * s

        return self.MAXRPM * s * np.exp(s * k * x) / (np.exp(k * 100))

    def DPS_RadS(self, x):

        return x * np.pi/180


    #Convert the angle to -pi tp pi range
    def pi_2_pi(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi
    

    def state_update(self, PWM_signal):
        '''
        Moving to the next step given the input u
        return : s
        '''
        u = [0, 0]
        k_l = 1
        k_r = 1

        # u[0] = self.PWM_to_RPM(float(PWM_signal[0]) + np.random.normal(0,self.PWM_std[0])) + np.random.normal(0,self.Wheel_std[0])
        # u[1] = self.PWM_to_RPM(float(PWM_signal[1]) + np.random.normal(0,self.PWM_std[1])) + np.random.normal(0,self.Wheel_std[1])
        self.wl = self.DPS_RadS(float(PWM_signal[0])) +  np.random.normal(0,self.Wheel_std[0])
        self.wr = self.DPS_RadS(float(PWM_signal[1])) + np.random.normal(0,self.Wheel_std[1])


        # self.wl = self.MAXRPM * (np.pi / 30) * (u[0] / 100)  # + np.random.normal(0, self.PWM_std[0])
        # self.wr = self.MAXRPM * (np.pi / 30) * (u[1] / 100)  # + np.random.normal(0, self.PWM_std[1])
        # v = u[0]
        # ommega = u[1]
        u = np.vstack((self.wl, self.wr))
        print("u is:", u)

        B = np.array([[np.cos(self.S[2, 0]), 0],
                      [np.sin(self.S[2, 0]), 0],
                      [0, 1]])

        Dynamixs = np.array([[self.diameter / 4, self.diameter / 4],
                             [-self.diameter / (2 * self.width), self.diameter / (2 * self.width)]])

        self.S = + self.S + (B @ Dynamixs @ u) * self.delta_t
        self.S[2,0] = self.pi_2_pi(self.S[2,0])

        return self.S

    def get_lidar(self):
        front_lidar = 0
        right_lidar = 0
        # x_pos = self.S[0]
        # y_pos = self.S[1]
        # theta_f = self.S[2]
        # theta_r = self.S[2] - math.pi / 2
        # length = self.room_length
        # width = self.room_width

        # front_point = [-1, -1]
        # right_point = [-1, -1]

        # # front sensor find coords
        # if math.sin(theta_f) == 0:
        #     front = [[0, y_pos], [width, y_pos]]
        # elif math.cos(theta_f) == 0:
        #     front = [[x_pos, 0], [x_pos, length]]
        # else:
        #     front = [[0, math.tan(theta_f) * (-1 * x_pos) + y_pos],
        #              [width, math.tan(theta_f) * (width - x_pos) + y_pos],
        #              [(-1 * y_pos) / math.tan(theta_f) + x_pos, 0],
        #              [(length - y_pos) / math.tan(theta_f) + x_pos, length]]

        # # front sensor find intersection
        # for coord in front:
        #     if (coord[0] >= 0 and coord[0] <= width and
        #             coord[1] >= 0 and coord[1] <= length and
        #             ((math.sin(theta_f) * (coord[0] - x_pos)) + (math.cos(theta_f) * (coord[1] - y_pos))) >= 0):
        #         front_point = coord

        # # right sensor find coords
        # if math.sin(theta_r) == 0:
        #     right = [[0, y_pos], [width, y_pos]]
        # elif math.cos(theta_r) == 0:
        #     right = [[x_pos, 0], [x_pos, length]]
        # else:
        #     right = [[0, math.tan(theta_r) * (-1 * x_pos) + y_pos],
        #              [width, math.tan(theta_r) * (width - x_pos) + y_pos],
        #              [(-1 * y_pos) / math.tan(theta_r) + x_pos, 0],
        #              [(length - y_pos) / math.tan(theta_r) + x_pos, length]]

        # # right sensor find intersection
        # for coord in right:
        #     if (coord[0] >= 0 and coord[0] <= width and
        #             coord[1] >= 0 and coord[1] <= length and
        #             ((math.sin(theta_r) * (coord[0] - x_pos)) + (math.cos(theta_r) * (coord[1] - y_pos))) >= 0):
        #         right_point = coord

        # get distances of coords
        # if front_point == [-1, -1] or right_point == [-1, -1]:
        #     raise ValueError("lidar: intersection not found!!! \n" + str(front_point) + "\n" + str(right_point))
        # front_lidar = math.sqrt((x_pos - front_point[0]) ** 2 + (y_pos - front_point[1]) ** 2)
        # right_lidar = math.sqrt((x_pos - right_point[0]) ** 2 + (y_pos - right_point[1]) ** 2)
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



