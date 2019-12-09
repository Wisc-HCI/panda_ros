__author__ = 'drakita'

import socket
from struct import *
import time


class UR5_dataReader:
    '''
    This class allows for 125Hz reading of all data from the ur5.
    First call update() to update data, then access data
    '''

    def __init__(self, sock=None, ip="10.130.229.132"):
        '''
        constructor
        :param socket: can optionally pass in a socket if this class is used in a larger framework
        :param ip: ip address of robot
        :return:
        '''
        self.ip = ip
        self.port = 30003
        if not sock == None:
            self.sock = sock
        else:
            HOST = ip
            PORT = 30003
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print self.sock
            self.sock.connect((HOST,PORT))


        #  variables
        self.messageSize = 1060
        self.time = 0.0 # time elapsed since the controller was started
        self.q_target = 6*[0.0] # target joint positions
        self.qd_target = 6*[0.0] # target joint velocities
        self.qdd_target = 6*[0.0] # target joint accelerations
        self.i_target = 6*[0.0] # target joint currents
        self.m_target = 6*[0.0] # target torques
        self.q_actual = 6*[0.0] # actual joint positions
        self.qd_actual = 6*[0.0] # actual joint velocities
        self.i_actual = 6*[0.0] # actual joint currents
        self.i_control = 6*[0.0] # joint control currents
        self.tool_vec_actual = 6*[0.0] # actual cartesian coordinates of the tool (x,y,z,rx,ry,rz)
        self.tcp_speed_actual = 6*[0.0] # actual speed of the tool given in cartesian coords
        self.tcp_force = 6*[0.0] # genearlized forces in the tcp
        self.tool_vec_target = 6*[0.0] # target cartesian coordinates of the tool (x,y,z,rx,ry,rz)
        self.tcp_speed_target = 6*[0.0] # target speed of the tool given in cartesian coords
        self.digital_input_bits = 0.0 # crrent state of the digital inputs
        self.motor_temps = 6*[0.0] # motor temperatures (celsius)
        self.controller_timer = 0.0 # controller realtime thread execution time
        self.test_value = 0.0 # a value used by UR software only
        self.robot_mode = 0.0 # robot mode
        self.joint_modes = 6*[0.0] # joint control modes
        self.safety_mode = 0.0
        self.tool_accel_values = 3*[0.0] # tool accelerometer values
        self.speed_scaling = 0.0 # speed scaling of the trajectory limiter
        self.lin_momentum_norm = 0.0 # norm of cartesian linear momentum
        self.v_main = 0.0 # masterboard: main voltage
        self.v_robot = 0.0 #masterboard:robot voltage (48v)
        self.i_robot = 0.0 # materboard: robot current
        self.v_actual = 6*[0.0] # actual joint voltages
        self.digital_outputs = 0.0 # digitial outputs
        self.program_state = 0.0 # program state


    def update(self):
        recv = self.sock.recv(1060)
        while not len(recv) == 1060:
            print len(recv)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip,self.port))
            recv = self.sock.recv(1060)

        format = "!idddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
        line =  unpack(format, recv)
        self.line = line
        idx = 0
        self.messageSize = line[idx]
        idx += 1
        self.time = line[idx]
        idx += 1
        self.q_target = line[idx:idx+6]
        idx += 6
        self.qd_target = line[idx:idx+6]
        idx += 6
        self.qdd_target = line[idx:idx+6]
        idx += 6
        self.i_target = line[idx:idx+6]
        idx += 6
        self.m_target = line[idx:idx+6]
        idx += 6
        self.q_actual = line[idx:idx+6]
        idx += 6
        self.qd_actual = line[idx:idx+6]
        idx += 6
        self.i_actual = line[idx:idx+6]
        idx += 6
        self.i_control = line[idx:idx+6]
        idx += 6
        self.tool_vec_actual = line[idx:idx+6]
        idx += 6
        self.tcp_speed_actual = line[idx:idx+6]
        idx += 6
        self.tcp_force = line[idx:idx+6]
        idx += 6
        self.tool_vec_target = line[idx:idx+6]
        idx += 6
        self.tcp_speed_target = line[idx:idx+6]
        idx += 6
        self.digital_input_bits = line[idx]
        idx += 1
        self.motor_temps = line[idx:idx+6]
        idx += 6
        self.controller_timer = line[idx]
        idx += 1
        self.test_value = line[idx]
        idx += 1
        self.robot_mode = line[idx]
        idx += 1
        self.joint_modes = line[idx:idx+6]
        idx += 6
        self.safety_mode = line[idx]
        idx += 1
        idx += 6 # empty
        self.tool_accel_values = line[idx:idx+3]
        idx += 3
        idx += 6 # empty
        self.speed_scaling = line[idx]
        idx += 1
        self.lin_momentum_norm = line[idx]
        idx += 1
        idx += 2 # empty
        self.v_main = line[idx]
        idx += 1
        self.v_robot = line[idx]
        idx += 1
        self.i_robot = line[idx]
        idx += 1
        self.v_actual = line[idx:idx+6]

if __name__ == '__main__':
    dr = UR5_dataReader()
    dr.update()
    import numpy as np
    import math as M

    print np.array(dr.q_actual) - np.array([-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 2])
    



