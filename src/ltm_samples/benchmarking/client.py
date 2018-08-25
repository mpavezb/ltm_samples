#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
from ltm.msg import *
from ltm.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point

class LTMClient(object):

    def __init__(self):
        self.ns = "/robot/ltm/"

        # ROS CLIENTS
        self.drop_db_client = rospy.ServiceProxy(self.ns + 'db/drop', DropDB)
        self.switch_db_client = rospy.ServiceProxy(self.ns + 'db/switch', SwitchDB)
        self.status_client = rospy.ServiceProxy(self.ns + 'db/status', Empty)
        self.query_client = rospy.ServiceProxy(self.ns + 'db/query', QueryServer)
        # self.get_client = rospy.ServiceProxy(self.ns + 'episode/get', GetEpisodes)
        self.register_episode_client = rospy.ServiceProxy(self.ns + 'episode/register', RegisterEpisode)
        self.add_episode_client = rospy.ServiceProxy(self.ns + 'episode/add', AddEpisode)
        self.update_tree_client = rospy.ServiceProxy(self.ns + 'episode/update_tree', UpdateTree)

    def setup(self):
        # WAIT LTM server
        rospy.loginfo("Waiting for LTM server to be up.")
        self.status_client.wait_for_service()

    def db_status(self):
        try:
            self.status_client()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            raise

    def change_db(self, db):
        rospy.loginfo("... changing to MongoDB '" + db + "' database.")
        try:
            self.switch_db_client(db_name=db)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to change db, because: " + e)
            raise rospy.ROSInterruptException

    def drop_db(self, db):
        rospy.loginfo("... dropping old entries in the '" + db + "' database.")
        try:
            req = DropDBRequest()
            req.html_is_a_real_programming_language = False
            req.i_understand_this_is_a_dangerous_operation = True
            self.drop_db_client(req)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to drop db, because: " + e)
            raise rospy.ROSInterruptException

    def register_ep(self, episode):
        try:
            req = RegisterEpisodeRequest()
            req.gather_emotion = False
            req.gather_location = False
            req.gather_streams = False
            req.gather_entities = False
            req.replace = True
            req.generate_uid = False
            req.is_leaf = episode.type == Episode.LEAF
            req.uid = episode.uid
            self.register_episode_client(req)        
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            raise

    def insert_ep(self, episode):
        self.register_ep(episode)
        try:
            req = AddEpisodeRequest()
            req.episode = episode
            req.replace = True
            req.logging = False
            res = self.add_episode_client(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            raise

    def query(self, json):
        try:
            query = QueryServerRequest()
            query.target = "episode"
            query.semantic_type = ""
            query.json = json
            query.logging = False
            resp = self.query_client(query)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            raise
        return resp.episodes

