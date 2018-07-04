#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random

# ROS
import rospy
from ltm_addons.msg import ImageStream
from ltm_addons.json_parser import JsonParser
from . import util


class ImageStreamFaker(object):

    def __init__(self):
        pass

    def generate(self):
        stream = ImageStream()
        stream.meta.uid = random.randint(0, 10)
        stream.meta.episode = stream.meta.uid
        stream.meta.start = rospy.Time.now()
        stream.meta.end = rospy.Time.now()
        stream.images = self.gen_images()
        return stream

    @staticmethod
    def gen_images(n_images=None):
        images = []
        if not n_images:
            # append from 1 up to 5 images
            n_images = random.randint(1, 5)
        for i in range(n_images):
            images.append(util.get_random_ros_image('place'))
        return images

    def build(self, uid, start, end, n_images):
        stream = ImageStream()
        stream.meta.uid = uid
        stream.meta.episode = uid
        stream.meta.start = JsonParser().str_to_ros_time(start)
        stream.meta.end = JsonParser().str_to_ros_time(end)
        stream.images = self.gen_images(n_images)
        return stream
