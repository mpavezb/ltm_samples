#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
import datetime 

# ROS
import rospy
from rospy import Time
from ltm.msg import *
from ltm.srv import *
from geometry_msgs.msg import Point


class EpisodeFaker(object):

    def __init__(self):
        pass

    @staticmethod
    def gen_date():
        year = random.randint(2010, 2020)
        month = random.randint(1, 12)
        day = random.randint(1, 28)
        hour = random.randint(0, 23)
        minute = random.randint(0, 59)
        second = random.randint(0, 59)
        return datetime.datetime(year, month, day, hour, minute, second)

    def gen_random_dates(self, size):
        duration_minutes = random.randint(10, 180) # 10 min up to 3 hours
        duration_seconds = duration_minutes * 60.0
        delta_seconds = duration_seconds / size

        epoch = datetime.datetime.utcfromtimestamp(0)
        start_seconds = (self.gen_date() - epoch).total_seconds()
        start = rospy.Time.from_sec(start_seconds)

        slices = list()
        for cnt in range(size):
            delta = rospy.Duration.from_sec(cnt * delta_seconds)
            slices.append(start + delta)
        return slices

    def gen_rand_locations(self):
        a = random.randint(0, 50)
        b = random.randint(0, 50)
        if a == b:
            b += 1
        location_a = "location_" + str(a)
        location_b = "location_" + str(b)
        return location_a, location_b


    def gen_where(self):
        position = Point()
        position.x = random.uniform(-20.0, 20.0)
        position.y = random.uniform(-20.0, 20.0)

        # generate Where field
        where = Where()
        where.position = position
        where.map_name = "map_" + str(random.randint(0, 20))
        where.frame_id = "/map"
        where.location = "location_" + str(random.randint(0, 100))
        where.area = "area_" + str(random.randint(0, 30))
        return where

    def gen_emotional_relevance(self):
        emotions = EmotionalRelevance()
        emotions.software = "FOO"
        emotions.software_version = "v1.0"
        emotions.registered_emotions = ["JOY", "TRUST", "FEAR", "SURPRISE", "SADNESS", "DISGUST", "ANGER", "ANTICIPATION"]
        values = [random.uniform(0.1, 1.0) for x in range(0, 8)]
        total = sum(values)
        emotions.registered_values = [x / total for x in values]
        val, idx = max((val, idx) for (idx, val) in enumerate(emotions.registered_values))
        emotions.value = val
        emotions.emotion = idx
        return emotions

    def gen_historical_relevance(self):
        historical = HistoricalRelevance()
        historical.value = random.uniform(0, 1.0)
        historical.last_update = Date()
        historical.next_update = Date()
        return historical

    def gen_relevance(self):
        relevance = Relevance()
        relevance.historical = self.gen_historical_relevance()
        relevance.emotional = self.gen_emotional_relevance()
        return relevance

    def gen_tags(self):
        n_tags = random.randint(1, 3)
        tags = []
        for it in range(n_tags):
            tags.append("tag_" + str(random.randint(0,1000)))
        return tags

    def gen_episode(self):
        ep = Episode()
        ep.uid = None                # OK
        ep.type = None               # OK
        ep.parent_id = None          # OK
        ep.tags = self.gen_tags()    # OK
        ep.children_tags = []        # OK
        ep.children_ids = []         # OK
        ep.info = Info()             # OK. Unused
        ep.what = What()             # TODO AUTO???
        ep.when = When()             # OK
        ep.where = Where()           # OK. Only children
        ep.relevance = Relevance()   # OK. Only children
        return ep

    def create_node(self, uid, parent_id, children):
        ep = self.gen_episode()
        ep.uid = uid
        ep.type = Episode.EPISODE
        ep.parent_id = parent_id

        # UPDATE FROM CHILDREN
        ep.when.start = rospy.Time.now()
        ep.when.end = rospy.Time(0)
        for child in children:
            ep.children_ids.append(child.uid)
            ep.children_tags += child.children_tags
            ep.children_tags += child.tags

            # when
            if ep.when.start > child.when.start:
                ep.when.start = child.when.start
            if ep.when.end < child.when.end:
                ep.when.end = child.when.end

        # unique items
        ep.children_tags = list(set(ep.children_tags))
        return ep

    def create_leaf(self, uid, parent_id, timestamps, start_idx, end_idx):
        ep = self.gen_episode()
        ep.uid = uid
        ep.type = Episode.LEAF
        ep.parent_id = parent_id
        ep.when.start = timestamps[start_idx]
        ep.when.end = timestamps[end_idx]
        ep.where = self.gen_where()
        ep.relevance = self.gen_relevance()
        return ep