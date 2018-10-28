#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import csv
import datetime
import rospy
import rospkg
from ltm_samples.fake.episodes import EpisodeFaker
from ltm_samples.benchmarking.client import LTMClient

class BenchmarkSuite(object):

    def __init__(self, test_name):
        self.test_name = test_name
        title = "LTM " + test_name + " test"
        rospy.loginfo(title)
        rospy.loginfo("="*len(title))
        
        self.output_folder = rospkg.RosPack().get_path('ltm_samples') + "/scripts/profile/results/"
        file_suffix = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.output_filename = self.output_folder + test_name + "_" + file_suffix + "_ouput" + ".csv"
        self.config_filename = self.output_folder + test_name + "_" + file_suffix + "_config" + ".txt"
        # self.output_filename = self.output_folder + test_name + "_ouput.csv"
        # self.config_filename = self.output_folder + test_name + "_config.txt"
        self.db_name = "test_" + test_name + "_db"

        self.faker = EpisodeFaker()
        self.ltm = LTMClient()

    def setup(self, configuration_str):
        rospy.loginfo("Setting up:")
        self.ltm.setup()
        self.ltm.change_db(self.db_name)
        self.ltm.drop_db(self.db_name)
        self.ltm.db_status()
        self.clear_output_file()
        self.write_config_file(configuration_str)

    def clear_output_file(self):
        open(self.output_filename, 'w').close()

    def write_config_file(self, configuration_str):
        with open(self.config_filename, 'w') as f:
            f.write("LTM " + self.test_name + " Parameters\n")
            f.write(" - start time: " + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "\n")
            f.write(" - configuration file: " + self.config_filename + "\n")
            f.write(" - results file: " + self.output_filename + "\n")
            f.write(configuration_str)
            f.write("\n")

    def write_row(self, row):
        with open(self.output_filename, 'a') as f:
            writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(row)

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
        self.ltm.insert_ep(ep01)
        self.ltm.insert_ep(ep02)
        self.ltm.insert_ep(ep03)
        self.ltm.insert_ep(ep04)
        self.ltm.insert_ep(ep05)
        self.ltm.insert_ep(ep06)
        self.ltm.insert_ep(ep07)
        self.ltm.insert_ep(ep08)
        self.ltm.insert_ep(ep09)
        self.ltm.insert_ep(ep10)

    def search_time(self, json):
        """returns elapsed milliseconds"""
        start = datetime.datetime.now()
        self.query(json)
        elapsed = datetime.datetime.now() - start
        return elapsed.total_seconds() * 1000.0

    def query(self, json):
        self.ltm.query(json)

    def status(self):
        self.ltm.db_status()

    def generate_checkpoints(self, max_value, max_delta):
        start = 10
        delta = 10
        value = start
        keep_increasing_delta = True
        checkpoints = []
        while value <= max_value:
            checkpoints.append(value)
            if keep_increasing_delta and value >= 10*delta:
                if (10*delta <= max_delta):
                    delta = 10*delta
                else:
                    keep_increasing_delta = False
            value += delta
        return checkpoints