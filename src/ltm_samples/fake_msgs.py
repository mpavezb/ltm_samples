#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
from datetime import date
from sensor_msgs.msg import Image
from ltm_samples.msg import *
from faker import Faker


class MsgGenerator(object):

    def __init__(self):
        self.fake = Faker()
        self.emotions = ["JOY", "ANGER", "ANTICIPATION", "DISGUST", "FEAR", "SADNESS", "SURPRISE", "TRUST"]
        self.stances = ["seated", "standing", "sleeping", "jumping", "walking"]

    def gen_human(self):
        human = Human()
        human.uid = random.randint(0, 10)
        human.name = str(self.fake.first_name())
        human.last_name = str(self.fake.last_name())
        human.genre = random.randint(0, 1)
        human.country = str(self.fake.country())
        human.city = str(self.fake.city())
        human.birthday.year = random.randint(1940, 2010)
        human.birthday.month = random.randint(1, 12)
        human.birthday.day = random.randint(1, 28)
        human.age = self.calculate_age(human.birthday)
        human.body = None
        human.face = None
        human.state = self.gen_human_state()
        return human

    def gen_human_state(self):
        state = HumanState()
        state.emotion = self.emotions[random.randint(0, len(self.emotions)-1)]
        state.stance = self.stances[random.randint(0, len(self.stances)-1)]
        return state

    def calculate_age(self, born):
        today = date.today()
        return today.year - born.year - ((today.month, today.day) < (born.month, born.day))

    def gen_object(self):
        obj = Object()
        obj.uid = random.randint(0, 10)
        return obj

    def gen_object_state(self):
        state = ObjectState()
        return state

    def gen_robot(self):
        robot = Robot()
        robot.uid = random.randint(0, 10)
        return robot

    def gen_robot_state(self):
        state = RobotState()
        return state

    def gen_place(self):
        place = Place()
        place.uid = random.randint(0, 10)
        return place


def main():
    gen = MsgGenerator()
    print gen.gen_human()


if __name__ == '__main__':
    main()