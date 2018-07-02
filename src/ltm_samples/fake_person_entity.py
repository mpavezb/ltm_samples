#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
from faker import Faker

# ROS
import rospy
from sensor_msgs.msg import Image
from ltm_samples.msg import PersonEntity
from . import util


class PersonEntityFaker(object):

    def __init__(self):
        self.fake = Faker()
        self.emotions = ["JOY", "ANGER", "ANTICIPATION", "DISGUST", "FEAR", "SADNESS", "SURPRISE", "TRUST"]
        self.stances = ["seated", "standing", "sleeping", "jumping", "walking"]

    def generate(self):
        person = PersonEntity()
        person.uid = random.randint(0, 10)
        person.name = str(self.fake.first_name())
        person.last_name = str(self.fake.last_name())
        person.genre = random.randint(0, 1)
        person.country = str(self.fake.country())
        person.city = str(self.fake.city())
        person.birthday.year = random.randint(1940, 2010)
        person.birthday.month = random.randint(1, 12)
        person.birthday.day = random.randint(1, 28)
        person.age = util.calculate_age(person.birthday)
        person.body = util.get_random_ros_image('body')
        person.face = util.get_random_ros_image('face')
        person.emotion = random.choice(self.emotions)
        person.stance = random.choice(self.stances)
        person.last_seen = rospy.Time.now()
        person.last_interacted = rospy.Time.now()
        person.is_nerd = (random.randint(0, 1) == 0)
        return person

    @staticmethod
    def null():
        person = PersonEntity()
        person.uid = 0
        person.name = ""
        person.last_name = ""
        person.genre = 2
        person.country = ""
        person.city = ""
        person.birthday.year = 0
        person.birthday.month = 0
        person.birthday.day = 0
        person.age = 0
        person.body = Image()
        person.face = Image()
        person.emotion = ""
        person.stance = ""
        person.last_seen = rospy.Time(0)
        person.last_interacted = rospy.Time(0)
        person.is_nerd = False
        return person

    @staticmethod
    def normalize(msg):
        return msg

    def is_field_null(self, msg, field):
        null_msg = self.null()
        value = msg.__getattribute__(field)
        null_value = null_msg.__getattribute__(field)
        return value == null_value
