#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random

# ROS
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from ltm_samples.msg import ObjectEntity
from . import util


class ObjectEntityFaker(object):

    def __init__(self):
        self.obj_types = ["food", "drink", "utensil", "furniture"]
        self.obj_names = ["beer", "cola", "fork", "spoon", "chair", "table", "meat", "greens"]
        self.place_names = ["kitchen", "living room", "dinner table", "bedroom", "garden", "entrance", "bathroom"]

    def generate(self):
        obj = ObjectEntity()
        obj.uid = random.randint(0, 10)
        obj.type = random.choice(self.obj_types)
        obj.name = random.choice(self.obj_names)
        obj.size = util.gen_point(0.05, 2.0)
        obj.image = util.get_random_ros_image('object')
        obj.map_name = "fake_map"
        obj.location.position = util.gen_point(-10.0, 10.0)
        obj.location.orientation.w = 1.0
        obj.clean = random.randint(0, 1)
        return obj

    @staticmethod
    def null():
        obj = ObjectEntity()
        obj.uid = 0
        obj.type = ""
        obj.name = ""
        obj.size = Point()
        obj.image = Image()
        obj.map_name = ""
        obj.location.position = Point()
        obj.clean = None
        return obj

    @staticmethod
    def normalize(msg):
        msg.clean = True
        return msg

    def is_field_null(self, msg, field):
        null_msg = self.null()
        value = msg.__getattribute__(field)
        null_value = null_msg.__getattribute__(field)
        return value == null_value