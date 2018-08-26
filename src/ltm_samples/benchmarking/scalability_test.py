#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import pymongo
import datetime

# ROS
import rospy
from ltm_samples.benchmarking.suite import BenchmarkSuite
from ltm_samples.benchmarking import queries

class ScalabilityTester(object):

    def __init__(self):
        self.suite = BenchmarkSuite("scalability")

        self.insertion_times = 100
        self.search_times = 100
        self.checkpoints = self.suite.generate_checkpoints(max_value=1000000, max_delta=10000)

        self.mongo = pymongo.MongoClient()
        self.db = self.mongo[self.suite.db_name]
        self.collection = self.db['episodes']

        config = ""
        config += " - insertion_times: " + str(self.insertion_times) + "\n"
        config += " - search_times: " + str(self.search_times) + "\n"
        config += " - time_units: [ms]\n"
        config += " - tree_episodes: 10\n"
        config += " - checkpoints: " + str(self.checkpoints) + "\n"
        config += " - #checkpoints: " + str(len(self.checkpoints)) + "\n"
        config += " - CSV header: row_count|it|dt_total_seconds|insertion_avg|simple_avg|complex_avg\n"
        self.suite.setup(config)

    def measure_search(self, json_func):
        ms_sum = 0
        for it in range(self.search_times):
            ms_sum += self.suite.search_time(json_func())
        ms_avg = ms_sum / (self.search_times + 0.0)
        return int(round(ms_avg))

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
            self.suite.gen_tree(it)
            insertion_elapsed = datetime.datetime.now() - insertion_start
            insertion_ms = insertion_elapsed.total_seconds() * 1000.0
            insertion_sum += insertion_ms
            insertion_count += 10

            it += 10
            if it in self.checkpoints:
                now = datetime.datetime.now()
                dt_total = now - start_time
                dt = now - last_checkpoint
                last_checkpoint = now
                rospy.loginfo(" - elapsed=" + str(dt_total) + ", dt=" + str(dt) + " - checkpoint: " + str(it) + " episodes.")
                
                # display db status on LTM server
                self.suite.status()

                # do measurements
                r1 = self.measure_search(queries.field_equals_int)
                r2 = self.measure_search(queries.array_contains_field)
                r3 = self.measure_search(queries.field_in_array)
                r4 = self.measure_search(queries.logical_or)
                r5 = self.measure_search(queries.nested_or_and)

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
                row.append(r1)
                row.append(r2)
                row.append(r3)
                row.append(r4)
                row.append(r5)
                row.append(ep_count)          # MongoDB: episode count (documents)
                row.append(stats['dataSize'])    # MongoDB: dataSize
                row.append(stats['indexSize'])   # MongoDB: indexSize
                row.append(stats['storageSize']) # MongoDB: storageSize
                row.append(stats['fileSize'])    # MongoDB: fileSize

                self.suite.write_row(row)


def main():
    try:
        rospy.init_node("ltm_scalability_testing")
        ScalabilityTester().execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
