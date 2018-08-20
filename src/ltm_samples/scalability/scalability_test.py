#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import pymongo
import csv
import random
import datetime

# ROS
import rospy
import rospkg
from rospy import Time
from ltm.msg import *
from ltm.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
from ltm_samples.faker.episodes import EpisodeFaker


class ScalabilityTester(object):

    def __init__(self):
        rospy.loginfo("LTM scalability test")
        rospy.loginfo("====================")
        self.test_db_name = "ltm_scalability_db"

        self.output_folder = rospkg.RosPack().get_path('ltm_samples') + "/scripts/profile/results/"
        self.file_suffix = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # self.output_filename = self.output_folder + "ltm_scalability_ouput_" + self.file_suffix + ".csv"
        # self.config_filename = self.output_folder + "ltm_scalability_config_" + self.file_suffix + ".txt"
        self.output_filename = self.output_folder + "ltm_scalability_ouput.csv"
        self.config_filename = self.output_folder + "ltm_scalability_config.txt"
        self.ns = "/robot/ltm/"
        self.insertion_times = 10
        self.search_times = 10
        # self.sleep_time = 0.0 # [seconds]
        self.checkpoints = self.generate_checkpoints(10000)
        self.checkpoints = self.generate_checkpoints(1000000)

        self.mongo = pymongo.MongoClient()
        self.db = self.mongo[self.test_db_name]
        self.collection = self.db['episodes']

        self.faker = EpisodeFaker()

        # ROS CLIENTS
        self.drop_db_client = rospy.ServiceProxy(self.ns + 'db/drop', DropDB)
        self.switch_db_client = rospy.ServiceProxy(self.ns + 'db/switch', SwitchDB)
        self.status_client = rospy.ServiceProxy(self.ns + 'db/status', Empty)
        self.query_client = rospy.ServiceProxy(self.ns + 'db/query', QueryServer)
        # self.get_client = rospy.ServiceProxy(self.ns + 'episode/get', GetEpisodes)
        self.register_episode_client = rospy.ServiceProxy(self.ns + 'episode/register', RegisterEpisode)
        self.add_episode_client = rospy.ServiceProxy(self.ns + 'episode/add', AddEpisode)
        self.update_tree_client = rospy.ServiceProxy(self.ns + 'episode/update_tree', UpdateTree)

        # WAIT SERVICES
        rospy.loginfo("Waiting for LTM server to be up.")
        self.add_episode_client.wait_for_service()
        self.update_tree_client.wait_for_service()

    def setup(self):
        rospy.loginfo("Setting up:")
        self.change_db(self.test_db_name)
        self.drop_db(self.test_db_name)
        self.db_status()
        self.clear_output_file()
        self.write_config_file()


    def write_config_file(self):
        with open(self.config_filename, 'w') as f:
            f.write("LTM Scalability Test Parameters\n")
            f.write(" - init_time: " + self.file_suffix + "\n")
            f.write(" - this_file: " + self.config_filename + "\n")
            f.write(" - output_file: " + self.output_filename + "\n")
            f.write(" - insertion_times: " + str(self.insertion_times) + "\n")
            f.write(" - search_times: " + str(self.search_times) + "\n")
            # f.write(" - sleep_time: " + str(self.sleep_time) + " [s]\n")
            f.write(" - time_units: [ms]\n")
            f.write(" - tree_episodes: 10\n")
            f.write(" - checkpoints: " + str(self.checkpoints) + "\n")
            f.write(" - #checkpoints: " + str(len(self.checkpoints)) + "\n")
            f.write(" - CSV header: row_count|it|dt_total_seconds|insertion_avg|simple_avg|complex_avg\n")
            f.write("\n")

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

    def gen_tree(self, uid):
        # rospy.loginfo(" - generating tree for root uid: " + str(uid))
        ts = self.faker.gen_random_dates(7)

        # leaves
        ep05 = self.faker.create_leaf(uid +  5, uid + 2, ts, 0, 1)    # Episode(uid=uid +  5) 
        ep06 = self.faker.create_leaf(uid +  6, uid + 2, ts, 1, 2)    # Episode(uid=uid +  6) 
        ep07 = self.faker.create_leaf(uid +  7, uid + 3, ts, 2, 3)    # Episode(uid=uid +  7) 
        ep08 = self.faker.create_leaf(uid +  8, uid + 3, ts, 3, 4)    # Episode(uid=uid +  8) 
        ep09 = self.faker.create_leaf(uid +  9, uid + 4, ts, 4, 5)    # Episode(uid=uid +  9) 
        ep10 = self.faker.create_leaf(uid + 10, uid + 4, ts, 5, 6)    # Episode(uid=uid + 10) 

        # nodes
        ep02 = self.faker.create_node(uid + 2, uid + 1, [ep05, ep06])    # Episode(uid=uid +  2) 
        ep03 = self.faker.create_node(uid + 3, uid + 1, [ep07, ep08])    # Episode(uid=uid +  3) 
        ep04 = self.faker.create_node(uid + 4, uid + 1, [ep09, ep10])    # Episode(uid=uid +  4) 
        ep01 = self.faker.create_node(uid + 1, 0, [ep02, ep03, ep04])    # Episode(uid=uid +  1) 

        # insert
        self.insert_ep(ep01)
        self.insert_ep(ep02)
        self.insert_ep(ep03)
        self.insert_ep(ep04)
        self.insert_ep(ep05)
        self.insert_ep(ep06)
        self.insert_ep(ep07)
        self.insert_ep(ep08)
        self.insert_ep(ep09)
        self.insert_ep(ep10)

    def generate_checkpoints(self, max_value):
        start = 10
        delta = 10
        value = start
        checkpoints = []
        while value <= max_value:
            checkpoints.append(value)
            if value >= 10*delta:
                delta = 10*delta
            value += delta
        return checkpoints

    def query(self, json):
        query = QueryServerRequest()
        query.target = "episode"
        query.semantic_type = ""
        query.json = json
        query.logging = False
        resp = self.query_client(query)
        return resp.episodes

    def search_json(self, json):
        """returns elapsed milliseconds"""
        start = datetime.datetime.now()
        self.query(json)
        elapsed = datetime.datetime.now() - start
        return elapsed.total_seconds() * 1000.0

    def measure_search(self, json_func):
        ms_sum = 0
        for it in range(self.search_times):
            ms_sum += self.search_json(json_func(self))
        ms_avg = ms_sum / (self.search_times + 0.0)
        return int(round(ms_avg))

    def test_search_simple(self):
        def json_func(self):
            location_a, location_b = self.faker.gen_rand_locations()
            return '{"where.location": { "$in": ["' + location_a + '", "' + location_b + '"] } }'
        return self.measure_search(json_func)

    def test_search_complex(self):
        def json_func(self):
            child_id = str(random.randint(1, 1000))
            start_id = random.randint(1, 1000)
            end_id = str(start_id + random.randint(1, 100))
            start_id = str(start_id)
            emotion = str(random.uniform(0, 1.0))

            json_1 = '"$or": [ {"children_ids": ' + child_id + '}, {"uid": { "$gte": ' + start_id + ', "$lte": ' + end_id + ' } } ]'
            json_2 = '"relevance.emotional.value": { "$gt": ' + emotion +' }'
            return '{' + json_1 + ', ' + json_2 + '}'
        return self.measure_search(json_func)

    def test_search(self):
        simple_avg = self.test_search_simple()
        complex_avg = self.test_search_complex()
        return simple_avg, complex_avg

    def clear_output_file(self):
        open(self.output_filename, 'w').close()

    def write_row(self, row):
        with open(self.output_filename, 'a') as f:
            writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(row)

    def execute(self):
        rospy.loginfo("Running:")
        rospy.logwarn(" ... Please DO NOT USE THIS MACHINE for other purposes.")

        start_time = datetime.datetime.now()
        last_checkpoint = start_time
        
        it = 0
        row_count = 0
        last = self.checkpoints[-1]
        insertion_sum = 0.0
        insertion_count = 0
        while it < last:

            insertion_start = datetime.datetime.now()
            self.gen_tree(it)
            insertion_elapsed = datetime.datetime.now() - insertion_start
            insertion_ms = insertion_elapsed.total_seconds() * 1000.0
            insertion_sum += insertion_ms
            insertion_count += 10

            it += 10
            if it in self.checkpoints:
                now = datetime.datetime.now()
                dt = now - last_checkpoint
                dt_total = now - start_time
                last_checkpoint = now
                rospy.loginfo(" - elapsed=" + str(dt_total) + ", dt=" + str(dt) + " - checkpoint: " + str(it) + " episodes.")
                
                # display db status on LTM server
                self.db_status()

                # do measurements
                # rospy.sleep(self.sleep_time)
                simple_avg, complex_avg = self.test_search()

                insertion_avg = int(round(insertion_sum / (0.0 + insertion_count)))
                insertion_sum = 0.0
                insertion_count = 0


                # MONGO DB STATS
                stats = self.db.command("dbstats")
                ep_count = self.collection.count()

                dt_total_seconds = int(round(dt_total.total_seconds()))
                row_count += 1
                row = list()
                row.append(row_count)         # row counter
                row.append(it)                # episodes
                row.append(dt_total_seconds)  # elapsed seconds
                row.append(insertion_avg)     # average insertion time 
                row.append(simple_avg)        # average simple search time
                row.append(complex_avg)       # average complex search time
                row.append(ep_count)          # MongoDB: episode count (documents)
                row.append(stats['dataSize'])    # MongoDB: dataSize
                row.append(stats['indexSize'])   # MongoDB: indexSize
                row.append(stats['storageSize']) # MongoDB: storageSize
                row.append(stats['fileSize'])    # MongoDB: fileSize

                self.write_row(row)


def main():
    try:
        rospy.init_node("ltm_scalability_testing")
        node = ScalabilityTester()
        node.setup()
        node.execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
