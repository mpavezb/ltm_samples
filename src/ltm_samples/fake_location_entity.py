#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
from ltm_samples.msg import PersonEntity


# print "---------------"
# print "place:"
# place = gen.gen_place()
# place.image = None
# print place

class LocationEntityFaker(object):

    def __init__(self):
        pass



    # def gen_place(self):
    #     place = Place()
    #     place.uid = random.randint(0, 10)
    #     place.map_name = "fake_map"
    #     place.name = random.choice(self.place_names)
    #     place.city = str(self.fake.city())
    #     place.country = str(self.fake.country())
    #     place.image = self.get_random_ros_image('place')
    #     place.center = self.gen_point(-5.0, 5.0)
    #     n_points = random.randint(3, 6)
    #     for k in range(n_points):
    #         place.shape.points.append(self.gen_point(-10.0, 10.0))
    #     return place
