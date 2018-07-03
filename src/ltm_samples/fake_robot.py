#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
import rospy
from geometry_msgs.msg import Point
from ltm.srv import *
from ltm.msg import *


class Robot(object):

    def __init__(self):
        rospy.loginfo("Your fake robot is ready to tell you lies.")

        # parameters
        self.where_locations = rospy.get_param("~where/locations", ["living_room", "kitchen"])
        self.where_areas = rospy.get_param("~where/areas", ["arena"])
        self.where_map = rospy.get_param("~where/map_name", "map")
        self.where_frame_id = rospy.get_param("~where/frame_id", "/map")
        self.relevance_emotional_software = rospy.get_param("~relevance/emotional/software", "emotion_engine")
        self.relevance_emotional_software_version = rospy.get_param("~relevance/emotional/software_version", "1.0.0")

        # ROS API
        rospy.Service('~get_location', GetEpisodes, self.service_get_where)
        rospy.Service('~get_emotion', GetEpisodes, self.service_get_emotion)

    def service_get_where(self, req):
        rospy.loginfo("Generating a fake location of the robot.")
        position = Point()
        position.x = random.uniform(-10.0, 10.0)
        position.y = random.uniform(-10.0, 10.0)

        # generate Where field
        where = Where()
        where.position = position
        where.map_name = self.where_map
        where.frame_id = self.where_frame_id
        where.location = random.choice(self.where_locations)
        where.area = random.choice(self.where_areas)

        # return response
        episode = Episode()
        episode.where = where
        res = GetEpisodesResponse()
        res.succeeded = True
        res.episodes.append(episode)
        return res

    def service_get_emotion(self, req):
        rospy.loginfo("Generating fake emotions for the robot.")
        emotions = EmotionalRelevance()
        emotions.software = self.relevance_emotional_software
        emotions.software_version = self.relevance_emotional_software_version
        emotions.registered_emotions = ["JOY", "TRUST", "FEAR", "SURPRISE", "SADNESS", "DISGUST", "ANGER", "ANTICIPATION"]
        values = [random.uniform(0.1, 1.0) for x in range(0, 8)]
        total = sum(values)
        emotions.registered_values = [x / total for x in values]
        val, idx = max((val, idx) for (idx, val) in enumerate(emotions.registered_values))
        emotions.value = val
        emotions.emotion = idx

        # return response
        episode = Episode()
        episode.relevance.emotional = emotions
        res = GetEpisodesResponse()
        res.succeeded = True
        res.episodes.append(episode)
        return res


def main():
    rospy.init_node("ltm_fake_entities")
    Robot()

    # Wait for ctrl-c to stop the application
    rospy.on_shutdown(lambda: rospy.logwarn("Ha det bra!."))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
