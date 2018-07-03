#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospkg
import rospy
import unittest

from ltm.srv import *
from ltm_addons.json_loader import JsonLoader


class LTMInterface(object):

    def __init__(self):
        # ROS client
        self.add_episode_client = rospy.ServiceProxy('/robot/ltm/add_episode', AddEpisode)
        self.query_client = rospy.ServiceProxy('/robot/ltm/query', QueryServer)
        self.get_episodes_client = rospy.ServiceProxy('/robot/ltm/get_episodes', GetEpisodes)
        self.drop_db_client = rospy.ServiceProxy('/robot/ltm/drop_db', DropDB)
        self.switch_db_client = rospy.ServiceProxy('/robot/ltm/switch_db', SwitchDB)

    def setup(self):
        # Wait for ROS services
        self.add_episode_client.wait_for_service()
        self.query_client.wait_for_service()
        self.get_episodes_client.wait_for_service()
        self.drop_db_client.wait_for_service()
        self.switch_db_client.wait_for_service()

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
        req = DropDBRequest()
        req.html_is_a_real_programming_language = False
        req.i_understand_this_is_a_dangerous_operation = True
        self.drop_db_client(req)

    def switch_database(self):
        self.switch_db_client(db_name="test")

    def insert_episode(self, episode):
        self.insert_episode(episode=episode)


class TestEpisodeQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        ltm = LTMInterface()
        ltm.setup()

        # Create an empty test DB
        ltm.switch_database()
        ltm.drop_database()

        # insert episodes from JSON
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('ltm_samples')
        loader = JsonLoader(source=pkg_path + "/samples/json/spr", ns="/robot/")
        loader.load()

    @classmethod
    def tearDownClass(cls):
        # # clean LTM server
        # ltm.drop_database()
        pass

    def setUp(self):
        self.ltm = LTMInterface()

    def tearDown(self):
        pass

    def test_eq_int(self):
        uids = self.ltm.query_episodes('{"uid": 0}')
        self.assertEqual(len(uids), 0)

        uids = self.ltm.query_episodes('{"uid": 2}')
        self.assertEqual(len(uids), 1)

    def test_eq_str(self):
        uids = self.ltm.query_episodes('{"where_frame_id": "/map"}')
        self.assertEqual(len(uids), 11)

        uids = self.ltm.query_episodes('{"where_frame_id": "unknown"}')
        self.assertEqual(len(uids), 0)

    def test_eq_bool(self):
        pass

    def test_eq_double(self):
        pass

    def test_str_array(self):
        uids = self.ltm.query_episodes('{"children_tags": "analyze_crowd"}')
        self.assertEqual(len(uids), 5)

    def test_int_array(self):
        uids = self.ltm.query_episodes('{"children_ids": 5}')
        self.assertEqual(len(uids), 1)

    def test_in_int(self):
        uids = self.ltm.query_episodes('{"relevance_emotional_emotion": { "$in": [0, 3, 2] } }')
        self.assertEqual(len(uids), 4)

    def test_in_str(self):
        uids = self.ltm.query_episodes('{"where_location": { "$in": ["living room", "lobby"] } }')
        self.assertEqual(len(uids), 5)

    def test_gt_lt_double(self):
        uids = self.ltm.query_episodes('{"relevance_emotional_value": { "$gt": 0.25 } }')
        self.assertEqual(len(uids), 6)

        uids = self.ltm.query_episodes('{"relevance_emotional_value": { "$lt": 0.2 } }')
        self.assertEqual(len(uids), 1)

    def test_logical_and(self):
        uids = self.ltm.query_episodes('{"relevance_emotional_value": { "$gt": 0.2, "$lt": 0.3 } }')
        self.assertEqual(len(uids), 4)

    def test_logical_or(self):
        json_1 = '{"relevance_emotional_value": { "$gt": 0.35 } }'
        json_2 = '{"children_tags": "analyze_crowd"}'
        json = '{"$or": [ ' + json_1 + ', ' + json_2 + ']}'
        uids = self.ltm.query_episodes(json)
        self.assertEqual(len(uids), 7)

    def test_complex(self):
        json_1 = '"$or": [ {"children_ids": 2}, {"uid": { "$gte": 4, "$lte": 10 } } ]'
        json_2 = '"relevance_emotional_value": { "$gt": 0.25 }'
        uids = self.ltm.query_episodes('{' + json_1 + ', ' + json_2 + '}')
        self.assertEqual(len(uids), 3)

    def test_invalid_json(self):
        pass

    def test_bad_fields(self):
        pass

    def test_invalid_operator(self):
        pass

    def test_invalid_characters(self):
        pass


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
