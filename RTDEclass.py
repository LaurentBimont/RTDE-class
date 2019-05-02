import sys
sys.path.append('..')
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import numpy as np
from threading import Thread
import time

class RTDE_urx(object):
    def __init__(self, ROBOT_HOST='192.168.68.23',
                 ROBOT_PORT=30004,
                 config_filename='control_loop_configuration.xml'):

        keep_running = True

        logging.getLogger().setLevel(logging.INFO)

        conf = rtde_config.ConfigFile(config_filename)

        state_names, state_types = conf.get_recipe('state')
        setp_names, setp_types = conf.get_recipe('setp')
        watchdog_names, watchdog_types = conf.get_recipe('watchdog')
        gripper_names, gripper_types = conf.get_recipe("gripper")

        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)

        self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)

        self.gripper = self.con.send_input_setup(gripper_names, gripper_types)

        ### Initialize variables of the register
        self.initialize()

        # start data synchronization
        if not self.con.send_start():
            sys.exit()


    def initialize(self):
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0

        self.gripper.input_int_register_1 = 0

        self.watchdog.input_int_register_0 = 0

+
    def setp_to_list(self):
        list = []
        for i in range(0, 6):
            list.append(self.setp.__dict__["input_double_register_%i" % i])
        return list

    def list_to_setp(self, list):
        for i in range(0, 6):
            self.setp.__dict__["input_double_register_%i" % i] = list[i]
        return self.setp

    def wait(self, target):
        state = self.con.receive()
        deltapose = np.array(state.actual_TCP_pose) - np.array(target)
        distance = (deltapose*deltapose).sum()
        if distance < 0.02:
            return True
        else:
            return False

    def movel(self, pos):
        while True:
            self.list_to_setp(pos)
            state = self.con.receive()
            if self.wait(pos):
                break
            if state.output_int_register_0 != 0:
                # print("ouloulou {}".format(setp))
                self.list_to_setp(pos)
                # send new setpoint
                self.con.send(self.setp)


    def open_gripper(self):
        self.gripper.input_int_register_1 = 2
        self.con.send(self.gripper)

    def close_gripper(self):
        self.gripper.input_int_register_1 = 1
        self.con.send(self.gripper)



class KeepAlive(Thread):
    def __init__(self, con, watchdog):
        super().__init__()
        self.con = con
        self.watchdog = watchdog
        self.daemon = True

    def run(self):

        while True:
            state = self.con.receive()
            self.con.send(self.watchdog)
            time.sleep(0.1)


if __name__=="__main__":

    robot = RTDE_urx()
    test = KeepAlive(robot.con, robot.watchdog)
    test.start()

    while True:

        robot.movel([0.00, -0.458, 0.300, 0.69, -3.04, 0.210])
        robot.close_gripper()
        robot.movel([0.30, -0.458, 0.300, 0.69, -3.04, 0.210])
        robot.open_gripper()
