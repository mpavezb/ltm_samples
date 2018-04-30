#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

from os import listdir
from os.path import isfile, join
import random
from datetime import date
import cv2

# ROS stuff
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from ltm_samples.msg import *
from faker import Faker


class MsgGenerator(object):

    def __init__(self):
        self.fake = Faker()
        self.emotions = ["JOY", "ANGER", "ANTICIPATION", "DISGUST", "FEAR", "SADNESS", "SURPRISE", "TRUST"]
        self.stances = ["seated", "standing", "sleeping", "jumping", "walking"]
        self.obj_types = ["food", "drink", "utensil", "furniture"]
        self.obj_names = ["beer", "cola", "fork", "spoon", "chair", "table", "meat", "greens"]
        self.place_names = ["kitchen", "living room", "dinner table", "bedroom", "garden", "entrance", "bathroom"]
        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('ltm_samples')

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
        human.body = self.get_random_ros_image('body')
        human.face = self.get_random_ros_image('face')
        human.state = self.gen_human_state()
        return human

    def gen_human_state(self):
        state = HumanState()
        state.emotion = random.choice(self.emotions)
        state.stance = random.choice(self.stances)
        return state

    def gen_object(self):
        obj = Object()
        obj.uid = random.randint(0, 10)
        obj.type = random.choice(self.obj_types)
        obj.name = random.choice(self.obj_names)
        obj.size = self.gen_point(0.05, 2.0)
        obj.image = self.get_random_ros_image('object')
        obj.state = self.gen_object_state()
        return obj

    def gen_object_state(self):
        state = ObjectState()
        state.map_name = "fake_map"
        state.location.position = self.gen_point(-10.0, 10.0)
        state.location.orientation.w = 1.0
        state.clean = True if random.randint(0, 1) == 1 else False
        return state

    def gen_robot(self):
        robot = Robot()
        robot.uid = random.randint(0, 10)
        robot.name = str(self.fake.first_name())
        robot.model = str(self.fake.license_plate())
        robot.type = str(self.fake.job())
        robot.company = str(self.fake.company())
        robot.components = ["arms", "head", "base", "torso", "lasers", "cameras", "microphone"]
        robot.skills = ["navigation", "manipulation", "hri", "perception", "ltm"]
        robot.birthday.year = random.randint(1940, 2010)
        robot.birthday.month = random.randint(1, 12)
        robot.birthday.day = random.randint(1, 28)
        robot.image = self.get_random_ros_image('robot')
        robot.state = self.gen_robot_state()
        return robot

    def gen_robot_state(self):
        state = RobotState()
        state.is_broken = True if random.randint(0, 1) == 1 else False
        state.emotion = random.choice(self.emotions)
        state.battery_level = random.uniform(0.1, 1.0)
        return state

    def gen_place(self):
        place = Place()
        place.uid = random.randint(0, 10)
        place.map_name = "fake_map"
        place.name = random.choice(self.place_names)
        place.city = str(self.fake.city())
        place.country = str(self.fake.country())
        place.image = self.get_random_ros_image('place')
        place.center = self.gen_point(-5.0, 5.0)
        n_points = random.randint(3, 6)
        for k in range(n_points):
            place.shape.points.append(self.gen_point(-10.0, 10.0))
        return place

    @staticmethod
    def gen_point(lower, upper):
        p = Point()
        p.x = random.uniform(lower, upper)
        p.y = random.uniform(lower, upper)
        p.z = random.uniform(lower, upper)
        return p

    @staticmethod
    def calculate_age(born):
        today = date.today()
        return today.year - born.year - ((today.month, today.day) < (born.month, born.day))

    def get_random_image(self, _type):
        try:
            target_dir = self.pkg_path + '/samples/images/' + _type
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

    def get_random_ros_image(self, _type):
        try:
            cv_image = self.get_random_image(_type)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except (CvBridgeError, TypeError) as e:
            print(">> Exception: " + str(e))
            return Image()
        return ros_image


def main():
    gen = MsgGenerator()
    human = gen.gen_human()
    human.body = None
    human.face = None
    print "---------------"
    print "human:"
    print human

    print "---------------"
    print "object:"
    obj = gen.gen_object()
    obj.image = None
    print obj

    print "---------------"
    print "robot:"
    robot = gen.gen_robot()
    robot.image = None
    print robot

    print "---------------"
    print "place:"
    place = gen.gen_place()
    place.image = None
    print place


if __name__ == '__main__':
    main()
