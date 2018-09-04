#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

from subprocess import check_output
import psutil
import datetime

# ROS
import rospy
from ltm_samples.benchmarking.suite import BenchmarkSuite
from ltm_samples.benchmarking import queries

class Measurement(object):

    def __init__(self, process):
        self.cpu = list() # percentage
        self.ram = list() # percentage
        self.ram_kB = list() # kB
        # self.io_reads = list() # bytes
        # self.io_writes = list() # bytes

        # init counter
        process.cpu_percent()

        # # init IO
        # io = process.io_counters()
        # self.init_io_reads = io.read_bytes/1024.0
        # self.init_io_writes = io.write_bytes/1024.0
        # self.start = datetime.datetime.now()

    def update(self, process):
        self.ram.append(process.memory_percent())
        self.ram_kB.append(process.memory_info().rss/1024.0)

        # remove zeros from CPU measurements
        cpu = process.cpu_percent()
        if cpu:
            self.cpu.append(cpu)

        # io = process.io_counters()
        # self.io_reads.append(io.read_bytes/1024.0)
        # self.io_writes.append(io.write_bytes/1024.0)

    def fmt(self, value, decimals=2):
        if not decimals: return int(value)
        multiplier = 1.0*pow(10, decimals)
        return int(value*multiplier)/multiplier

    def reduce(self):
        cpu_max = 0.0
        cpu_avg = 0.0
        ram_avg = 0.0
        ram_kB_avg = 0.0

        if self.cpu:
            cpu_max = self.fmt(max(self.cpu))
            cpu_avg = self.fmt(sum(self.cpu) / float(len(self.cpu)))
            
        if self.ram:
            ram_avg = self.fmt(sum(self.ram) / float(len(self.ram)))

        if self.ram_kB:
            ram_kB_avg = self.fmt(sum(self.ram_kB) / float(len(self.ram_kB)), 0)

        # ram_MB_avg = self.fmt(ram_kB_avg / 1024.0, 1)
        # rospy.loginfo("[cpu]: n=" + str(len(self.cpu)) + ", max=" + str(cpu_max) + ", avg=" + str(cpu_avg))
        # rospy.loginfo("[ram]: n=" + str(len(self.ram)) + ", avg=" + str(ram_avg) + "%, " + str(ram_kB_avg) + " kB / " + str(ram_MB_avg) + " MB")
        return [cpu_max, cpu_avg, ram_avg, ram_kB_avg]
        

class EfficiencyTester(object):

    def __init__(self):
        self.suite = BenchmarkSuite("efficiency")

        # self.sleep_time = 5.0 # [s]
        self.sleep_time = 0.0 # [s]
        self.ltm_pid = self.get_pid("ltm_server")
        self.ltm_process = psutil.Process(self.ltm_pid)
        self.mongo_pid = self.get_pid("mongod")
        self.mongo_process = psutil.Process(self.mongo_pid)

        self.episodes = [100, 1000, 10000, 50000, 100000]
        self.rates = map(lambda x: 25*(x+1), range(40)) # 25,50,...,1000
        self.start_time = datetime.datetime.now()
        self.min_measurement_queries = 5
        self.min_measurement_time = 60 # [s]
        self.measurement_interval = 3.0 # [s]

        config = ""
        config += " - start_time: " + str(self.start_time) + "\n"
        config += " - ltm pid: " + str(self.ltm_pid) + "\n"
        config += " - mongo pid: " + str(self.mongo_pid) + "\n"
        config += " - episodes: " + str(self.episodes) + "\n"
        config += " - rates [qpm]: " + str(self.rates) + "\n"
        config += " - n_episodes: " + str(len(self.episodes)) + "\n"
        config += " - n_rates: " + str(len(self.rates)) + "\n"
        config += " - min_measurement_time: " + str(self.min_measurement_time) + " [s]\n"
        config += " - min_measurement_queries: " + str(self.min_measurement_queries) + " [s]\n"
        config += " - measurement_interval: " + str(self.measurement_interval) + " [s]\n"
        config += " - queries: " + "q1:field_equals_int" + "\n"
        config += " - queries: " + "q2:array_contains_field" + "\n"
        config += " - queries: " + "q3:field_in_array" + "\n"
        config += " - queries: " + "q4:logical_or" + "\n"
        config += " - queries: " + "q5:nested_or_and" + "\n"
        self.suite.setup(config)

    def get_pid(self, name):
        return int(check_output(["pidof", "-s", name]))

    def test_base(self):
        pass

    def test_query(self, qpm, name, json_func):
        elapsed_from_init = datetime.datetime.now() - self.start_time
        rospy.loginfo("   - elapsed=" + str(elapsed_from_init) + "[s], query=" + name + ".")

        # TODO: OJO CON TRANSIENTES Y CAMBIOS ENTRE CADA QUERY!
        rospy.sleep(self.sleep_time) # sleep a moment to recover base state

        ltm = Measurement(self.ltm_process)
        mongo = Measurement(self.mongo_process)
        
        times = 0
        qps = qpm/60.0
        rate = rospy.Rate(qps)
        start = datetime.datetime.now()
        last_measurement = datetime.datetime.now()
        while not rospy.is_shutdown():
            elapsed = (datetime.datetime.now() - start).total_seconds()
            times += 1
            if times > self.min_measurement_queries and elapsed > self.min_measurement_time:
                break

            # perform query
            self.suite.search_time(json_func())

            # measure fields
            now = datetime.datetime.now()
            elapsed_time = (now - last_measurement).total_seconds()
            if elapsed_time > self.measurement_interval:
                last_measurement = now

                ltm.update(self.ltm_process)
                mongo.update(self.mongo_process)

            rate.sleep()

        row = list()
        row.extend(ltm.reduce())
        row.extend(mongo.reduce())
        return row


    def execute(self):
        rospy.loginfo("Running:")
        rospy.logwarn(" ... Please DO NOT USE THIS MACHINE for other purposes.")

        it = 0
        row_count = 0
        last_episode = self.episodes[-1]

        while it < last_episode:

            # TODO: MEASURE INSERTION EFFICIENCY
            self.suite.gen_tree(it)
            it += 10
            if it not in self.episodes:
                continue

            # TODO: measure insertion resource usage by creating labeled episodes and then deleting them.

            now = datetime.datetime.now()
            rospy.loginfo(" > CHECKPOINT: " + str(it) + " episodes. (elapsed=" + str(now - self.start_time) + ")")
            
            # display db status on LTM server
            self.suite.status()

            # DO MEASUREMENTS
            for qpm in self.rates:
                now = datetime.datetime.now()
                elapsed = now - self.start_time
                rospy.loginfo(" - RATE: " + str(qpm) + " [queries/min] (elapsed=" + str(elapsed) + ")")

                r1 = self.test_query(qpm, "q1:field_equals_int", queries.field_equals_int)
                r2 = self.test_query(qpm, "q2:array_contains_field", queries.array_contains_field)
                r3 = self.test_query(qpm, "q3:field_in_array", queries.field_in_array)
                r4 = self.test_query(qpm, "q4:logical_or", queries.logical_or)
                r5 = self.test_query(qpm, "q5:nested_or_and", queries.nested_or_and)
                if rospy.is_shutdown():
                    exit()
                
                now = datetime.datetime.now()
                elapsed = now - self.start_time
                elapsed_seconds = int(round(elapsed.total_seconds()))
                row_count += 1
                row = list()
                row.append(row_count)
                row.append(it)
                row.append(qpm)
                row.append(elapsed_seconds)
                row.extend(r1)
                row.extend(r2)
                row.extend(r3)
                row.extend(r4)
                row.extend(r5)
                self.suite.write_row(row)


def main():
    try:
        rospy.init_node("ltm_efficiency_testing")
        EfficiencyTester().execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
