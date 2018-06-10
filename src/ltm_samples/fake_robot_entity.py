#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
from ltm_samples.msg import PersonEntity


# print "---------------"
# print "robot:"
# robot = gen.gen_robot()
# robot.image = None
# print robot



class RobotEntityFaker(object):

    def __init__(self):
        pass


    # def gen_robot(self):
    #     robot = Robot()
    #     robot.uid = random.randint(0, 10)
    #     robot.name = str(self.fake.first_name())
    #     robot.model = str(self.fake.license_plate())
    #     robot.type = str(self.fake.job())
    #     robot.company = str(self.fake.company())
    #     robot.components = ["arms", "head", "base", "torso", "lasers", "cameras", "microphone"]
    #     robot.skills = ["navigation", "manipulation", "hri", "perception", "ltm"]
    #     robot.birthday.year = random.randint(1940, 2010)
    #     robot.birthday.month = random.randint(1, 12)
    #     robot.birthday.day = random.randint(1, 28)
    #     robot.image = self.get_random_ros_image('robot')
    #     robot.state = self.gen_robot_state()
    #     return robot

    # def gen_robot_state(self):
    #     state = RobotState()
    #     state.is_broken = True if random.randint(0, 1) == 1 else False
    #     state.emotion = random.choice(self.emotions)
    #     state.battery_level = random.uniform(0.1, 1.0)
    #     return state
