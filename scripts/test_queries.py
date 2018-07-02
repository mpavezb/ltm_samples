#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
import unittest

from std_srvs.srv import Empty
from ltm.msg import Episode
from ltm.srv import *
from ltm_samples.msg import PersonEntity

# Tests:
# - eq, ne: string, int, double, bool
# - gt, lt, gte, lte
# - in, nin
# - AND operations
#


class LTMInterface(object):

    def __init__(self):
        # ROS client
        self.add_episode_client = rospy.ServiceProxy('/robot/ltm/add_episode', AddEpisode)
        self.query_client = rospy.ServiceProxy('/robot/ltm/query', QueryServer)
        self.get_episodes_client = rospy.ServiceProxy('/robot/ltm/get_episodes', GetEpisodes)
        self.drop_db_client = rospy.ServiceProxy('/robot/ltm/drop_db', Empty)

    def setup(self):
        # Wait for ROS services
        self.add_episode_client.wait_for_service()
        self.query_client.wait_for_service()
        self.get_episodes_client.wait_for_service()
        self.drop_db_client.wait_for_service()

    def query_episodes(self, json):
        query = QueryServerRequest()
        query.target = "episode"
        query.semantic_type = ""
        query.json = json
        resp = self.query_client(query)
        return resp.uids

    def query_people(self, json):
        query = QueryServerRequest()
        query.target = "entity"
        query.semantic_type = "person"
        query.json = json
        resp = self.query_client(query)
        return resp.uids

    def drop_database(self):
        self.drop_db_client()
        return


class TestEpisodeQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        ltm = LTMInterface()
        ltm.setup()

        # TODO: enable testing mode.. do not drop user database!.. just use 'ltm_test_db' MongoDB
        ltm.drop_database()

        # insert episodes

    @classmethod
    def tearDownClass(cls):
        # clean LTM server
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_equal_uint32(self):
        ltm = LTMInterface()
        json = '{"uid": 0}'
        uids = ltm.query_episodes(json)
        # self.assertEqual(len(uids), 1, 'Duplicate or missing uid')
        # self.assertEqual(uids[0], 0)

    def test_equal_string(self):
        ltm = LTMInterface()

        json = '{"where_frame_id": "/map"}'
        uids = ltm.query_episodes(json)
        # self.assertNotEqual(len(uids), 0)

        json = '{"where_frame_id": "unknown"}'
        uids = ltm.query_episodes(json)
        # self.assertEqual(len(uids), 0)

    def test_equal_bool(self):
        pass

    def test_invalid(self):
        pass

    def test_str_array(self):
        ltm = LTMInterface()
        json = '{"children_tags": "analyze_crowd"}'
        uids = ltm.query_episodes(json)
        # self.assertNotEqual(len(uids), 0)

    def test_int_array(self):
        ltm = LTMInterface()
        json = '{"children_ids": 1679966153}'
        uids = ltm.query_episodes(json)
        # self.assertNotEqual(len(uids), 0)

    def test_in_int(self):
        ltm = LTMInterface()
        json = '{"relevance_emotional_emotion": { "$in": [0, 3, 2] } }'
        uids = ltm.query_episodes(json)

    def test_in_str(self):
        ltm = LTMInterface()
        json = '{"where_location": { "$in": ["kitchen", "hallway"] } }'
        uids = ltm.query_episodes(json)

    def test_gt_lt_double(self):
        ltm = LTMInterface()
        json = '{"relevance_emotional_value": { "$gt": 0.19 } }'
        uids = ltm.query_episodes(json)

        ltm = LTMInterface()
        json = '{"relevance_emotional_value": { "$lt": 0.2 } }'
        uids = ltm.query_episodes(json)

        ltm = LTMInterface()
        json = '{"relevance_emotional_value": { "$gt": 0.19, "$lt": 0.2 } }'
        uids = ltm.query_episodes(json)


class TestPeopleQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # insert episodes
        # insert entities
        pass

    @classmethod
    def tearDownClass(cls):
        # clean LTM server
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_equal_uint32(self):
        pass

    def test_equal_string(self):
        pass

    def test_equal_double(self):
        pass

    def test_equal_bool(self):
        ltm = LTMInterface()
        json = '{"is_nerd": true}'
        uids = ltm.query_people(json)
        self.assertNotEqual(len(uids), 1)


if __name__ == '__main__':
    rospy.init_node('test_ltm_queries')
    try:
        unittest.main()
    except rospy.ROSInterruptException:
        pass
