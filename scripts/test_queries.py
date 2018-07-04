#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospkg
import rospy
import unittest

from ltm_addons.json_loader import JsonLoader
from ltm_samples.fake_image_streams import ImageStreamFaker

from ltm.srv import *
from ltm_addons.srv import ImageStreamSrv, ImageStreamSrvRequest
from ltm_samples.srv import PersonEntitySrv, PersonEntitySrvRequest


class LTMInterface(object):

    def __init__(self):
        # self.get_client = rospy.ServiceProxy('/robot/ltm/episode/get', GetEpisodes)
        self.query_client = rospy.ServiceProxy('/robot/ltm/db/query', QueryServer)
        self.drop_db_client = rospy.ServiceProxy('/robot/ltm/db/drop', DropDB)
        self.switch_db_client = rospy.ServiceProxy('/robot/ltm/db/switch', SwitchDB)

    def setup(self):
        # self.get_client.wait_for_service()
        self.query_client.wait_for_service()
        self.drop_db_client.wait_for_service()
        self.switch_db_client.wait_for_service()

    def query(self, json):
        query = QueryServerRequest()
        query.target = "episode"
        query.semantic_type = ""
        query.json = json
        resp = self.query_client(query)
        return resp.episodes

    def drop_database(self):
        req = DropDBRequest()
        req.html_is_a_real_programming_language = False
        req.i_understand_this_is_a_dangerous_operation = True
        self.drop_db_client(req)

    def switch_to_test_database(self):
        self.switch_db_client(db_name="test")

    def switch_to_default_database(self):
        self.switch_db_client(db_name="ltm_db")

    def load_fixtures(self):
        # Load episodes from JSON
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('ltm_samples')
        loader = JsonLoader(source=pkg_path + "/samples/json/spr", ns="/robot/", logging=False)
        loader.load()


class StreamInterface(object):

    def __init__(self):
        self.add_client = rospy.ServiceProxy('/robot/ltm/stream/images/add', ImageStreamSrv)
        # self.get_client = rospy.ServiceProxy('/robot/ltm/stream/images/get', ImageStreamSrv)
        self.query_client = rospy.ServiceProxy('/robot/ltm/db/query', QueryServer)

    def setup(self):
        self.add_client.wait_for_service()
        # self.get_client.wait_for_service()
        self.query_client.wait_for_service()

    def query(self, json):
        query = QueryServerRequest()
        query.target = "stream"
        query.semantic_type = "images"
        query.json = json
        resp = self.query_client(query)
        return resp.streams[0].uids

    def insert(self, stream):
        req = ImageStreamSrvRequest()
        req.msg = stream
        self.add_client(req)

    def load_fixtures(self):
        faker = ImageStreamFaker()
        # times obtained from JSON samples
        # TODO: automate time recollection (from episodes or JSON)
        self.insert(faker.build(5, "UTC_2018/04/20_18:00:00.123456", "UTC_2018/04/20_18:00:30.456123", 1))
        self.insert(faker.build(6, "UTC_2018/04/20_18:00:31.789", "UTC_2018/04/20_18:01:12.123", 2))
        self.insert(faker.build(7, "UTC_2018/04/20_18:01:12.123456789", "UTC_2018/04/20_18:02:00.987654321", 5))
        self.insert(faker.build(9, "UTC_2018/04/20_18:02:00.0", "UTC_2018/04/20_18:02:45.0", 6))
        self.insert(faker.build(10, "UTC_2018/04/20_18:02:45.0", "UTC_2018/04/20_18:03:00.0", 3))


class PeopleInterface(object):

    def __init__(self):
        self.add_client = rospy.ServiceProxy('/robot/ltm/entity/people/add', PersonEntitySrv)
        # self.get_client = rospy.ServiceProxy('/robot/ltm/entity/people/get', PersonEntitySrv)
        self.query_client = rospy.ServiceProxy('/robot/ltm/db/query', QueryServer)

    def setup(self):
        self.add_client.wait_for_service()
        # self.get_client.wait_for_service()
        self.query_client.wait_for_service()

    def query(self, json):
        query = QueryServerRequest()
        query.target = "entity"
        query.semantic_type = "people"
        query.json = json
        resp = self.query_client(query)
        return resp.entities[0].uids

    def insert(self, person):
        req = PersonEntitySrvRequest()
        req.msg = person
        self.add_client(req)

    def load_fixtures(self):
        pass


class SetupCases(object):

    @staticmethod
    def execute():
        ltm = LTMInterface()
        stream = StreamInterface()

        # setup
        ltm.setup()
        stream.setup()

        # Create an empty test DB
        ltm.switch_to_test_database()
        ltm.drop_database()

        # load fixtures
        ltm.load_fixtures()
        stream.load_fixtures()

    @staticmethod
    def destroy():
        ltm = LTMInterface()
        ltm.switch_to_default_database()


class TestEpisodeQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        SetupCases().execute()

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
        uids = self.ltm.query('{"uid": 0}')
        self.assertEqual(len(uids), 0)

        uids = self.ltm.query('{"uid": 2}')
        self.assertEqual(len(uids), 1)

    def test_eq_str(self):
        uids = self.ltm.query('{"where.frame_id": "/map"}')
        self.assertEqual(len(uids), 11)

        uids = self.ltm.query('{"where.frame_id": "unknown"}')
        self.assertEqual(len(uids), 0)

    def test_eq_bool(self):
        # TODO
        pass

    def test_eq_double(self):
        # TODO
        pass

    def test_str_array(self):
        uids = self.ltm.query('{"children_tags": "analyze_crowd"}')
        self.assertEqual(len(uids), 5)

    def test_int_array(self):
        uids = self.ltm.query('{"children_ids": 5}')
        self.assertEqual(len(uids), 1)

    def test_in_int(self):
        uids = self.ltm.query('{"relevance.emotional.emotion": { "$in": [0, 3, 2] } }')
        self.assertEqual(len(uids), 4)

    def test_in_str(self):
        uids = self.ltm.query('{"where.location": { "$in": ["living room", "lobby"] } }')
        self.assertEqual(len(uids), 5)

    def test_gt_lt_double(self):
        uids = self.ltm.query('{"relevance.emotional.value": { "$gt": 0.25 } }')
        self.assertEqual(len(uids), 6)

        uids = self.ltm.query('{"relevance.emotional.value": { "$lt": 0.2 } }')
        self.assertEqual(len(uids), 1)

    def test_logical_and(self):
        uids = self.ltm.query('{"relevance.emotional.value": { "$gt": 0.2, "$lt": 0.3 } }')
        self.assertEqual(len(uids), 4)

    def test_logical_or(self):
        json_1 = '{"relevance.emotional.value": { "$gt": 0.35 } }'
        json_2 = '{"children_tags": "analyze_crowd"}'
        json = '{"$or": [ ' + json_1 + ', ' + json_2 + ']}'
        uids = self.ltm.query(json)
        self.assertEqual(len(uids), 7)

    def test_complex(self):
        json_1 = '"$or": [ {"children_ids": 2}, {"uid": { "$gte": 4, "$lte": 10 } } ]'
        json_2 = '"relevance.emotional.value": { "$gt": 0.25 }'
        uids = self.ltm.query('{' + json_1 + ', ' + json_2 + '}')
        self.assertEqual(len(uids), 3)

    def test_find_by_time(self):
        # episodes after timestamp
        uids = self.ltm.query('{"when.start": { $gt: 1524247000 }}')
        self.assertEqual(len(uids), 5)

        # episodes before timestamp
        uids = self.ltm.query('{"when.end": { $lt: 1524247321 }}')
        self.assertEqual(len(uids), 4)

        # episodes containing timestamp
        uids = self.ltm.query('{"when.start": { $lt: 1524247320}, "when.end": { $gt: 1524247320} }')
        self.assertEqual(len(uids), 7)

        # episodes in temporal range
        init = "1524247240"
        end = "1524247350"
        q1 = '{"when.start": { $lt: ' + end + ', $gt: ' + init + '}}'
        q2 = '{"when.end": { $gt: ' + init + ', $lt: ' + end + '}}'
        uids = self.ltm.query('{ "$or": [ ' + q1 + ', ' + q2 + ']}')
        self.assertEqual(len(uids), 4)

    def test_invalid_json(self):
        # TODO
        pass

    def test_bad_fields(self):
        # TODO
        pass

    def test_invalid_operator(self):
        # TODO
        pass

    def test_invalid_characters(self):
        # TODO
        pass

    # def test_equal_bool(self):
    #     ltm = LTMInterface()
    #     json = '{"is_nerd": true}'
    #     uids = ltm.query_people(json)
    #     self.assertNotEqual(len(uids), 1)


class TestStreamQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        SetupCases().execute()

    def setUp(self):
        self.ltm = StreamInterface()

    def tearDown(self):
        pass

    def test_find_by_uid(self):
        uids = self.ltm.query('{"uid": 5}')
        self.assertEqual(len(uids), 1)

        uids = self.ltm.query('{"uid": 0}')
        self.assertEqual(len(uids), 0)

    def test_find_by_images(self):
        uids = self.ltm.query('{"images": 0}')
        self.assertEqual(len(uids), 0)

        uids = self.ltm.query('{"images": 5}')
        self.assertEqual(len(uids), 1)

        uids = self.ltm.query('{"images": { "$gt": 3 }}')
        self.assertEqual(len(uids), 2)


class TestPeopleQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        SetupCases().execute()

    def setUp(self):
        self.ltm = PeopleInterface()

    def tearDown(self):
        pass

    # def test_find_by_uid(self):
    #     uids = self.ltm.query('{"uid": 5}')
    #     self.assertEqual(len(uids), 1)
    #
    #     uids = self.ltm.query('{"uid": 0}')
    #     self.assertEqual(len(uids), 0)
    #
    # def test_find_by_images(self):
    #     uids = self.ltm.query('{"images": 0}')
    #     self.assertEqual(len(uids), 0)
    #
    #     uids = self.ltm.query('{"images": 5}')
    #     self.assertEqual(len(uids), 1)
    #
    #     uids = self.ltm.query('{"images": { "$gt": 3 }}')
    #     self.assertEqual(len(uids), 2)


if __name__ == '__main__':
    rospy.init_node('test_ltm_queries')
    try:
        unittest.main()
    except rospy.ROSInterruptException:
        pass
