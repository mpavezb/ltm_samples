#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
import cv2
import rospkg
from os import listdir
from os.path import isfile, join
from datetime import date

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def gen_point(lower, upper):
    p = Point()
    p.x = random.uniform(lower, upper)
    p.y = random.uniform(lower, upper)
    p.z = random.uniform(lower, upper)
    return p


def calculate_age(born):
    today = date.today()
    return today.year - born.year - ((today.month, today.day) < (born.month, born.day))


def get_random_image(_type):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('ltm_samples')
    target_dir = pkg_path + '/samples/images/' + _type
    try:
        files = [f for f in listdir(target_dir) if isfile(join(target_dir, f)) and f.endswith('.jpg')]
        rand_id = random.randint(0, len(files)-1)
        filename = join(target_dir, files[rand_id])
        image = cv2.imread(filename, cv2.IMREAD_COLOR)
    except OSError as e:
        print(">> No .jpg image files were found on: " + target_dir)
        print(">> Please download some images (Read the ltm tutorials)")
        print(">> Exception: " + str(e))
        return Image()
    return image


def get_random_ros_image(_type):
    try:
        bridge = CvBridge()
        cv_image = get_random_image(_type)
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    except (CvBridgeError, TypeError) as e:
        print(">> Exception: " + str(e))
        return Image()
    return ros_image
