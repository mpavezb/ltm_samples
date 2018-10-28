#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import math
import shutil
import tarfile
import os
import pymongo
import rospy
from ltm_samples.benchmarking.client import LTMClient
from ltm_samples.fake.episodes import EpisodeFaker

class Record(object):

    def __init__(self, name, chunk_size, preffix, path):
        self.name = name
        self.chunk_size = chunk_size
        self.preffix = preffix
        self.path = path
        self.missing = set()
        self.found = set()

    def to_eps(self, n):
        return n * self.chunk_size

    def to_n(self, eps):
        return int(math.floor(eps / self.chunk_size))

    def parse_files(self):
        if not os.path.isdir(self.path):
            rospy.logwarn(" - " + self.name + " output directory not found ... creating: " + self.path)
            os.makedirs(self.path)
        rospy.loginfo(" - " + self.name + " output directory is: " + self.path)

        # get compressed records
        for f in os.listdir(self.path):
            if f.startswith(self.preffix) and f.endswith(".tar.gz"):
                # TODO: eliminar archivos corruptos
                no_preffix = f.replace(self.preffix, "")
                no_ext = no_preffix.replace(".tar.gz", "")
                no_chunk = no_ext[0:4]
                number = int(float(no_chunk))
                self.found.add(number)
        rospy.loginfo(" - Found %d %s." % (len(self.found), self.name))

    def compute_missing(self, eps):
        last = self.to_n(eps)
        complete = set(range(1, last + 1))
        return complete - self.found

    def is_checkpoint(self, eps):
        return eps % self.chunk_size == 0

    def gen_name(self, eps):
        n = self.to_n(eps)
        return "%s%04d_%d" % (self.preffix, n, self.chunk_size)


class BenchmarkDBGenerator(object):

    def __init__(self):
        self.db_name = "benchmarking_db"
        self.gen_path = "/home/mpavezb/workspaces/ltm_benchmarking"
        self.deltas = Record("delta record", 5000, "delta_", self.gen_path)
        self.fulls = Record("full record", 100000, "full_", self.gen_path)
        self.max_episodes = 1000000

        if self.deltas.chunk_size % 10 != 0:
            rospy.logerr("DELTA_SIZE MUST BE DIVISIBLE BY 10")
            exit()

        if self.deltas.chunk_size % self.deltas.chunk_size != 0:
            rospy.logerr("FULL_SIZE MUST BE DIVISIBLE BY DELTA_SIZE")
            exit()
        
        # LTM client
        self.ltm = LTMClient()
        self.ltm.setup()
        self.ltm.change_db(self.db_name)
        self.faker = EpisodeFaker()

        # MongoDB
        # ASUME QUE EPS EN BASE DE DATOS SIEMPRE SON CONSECUTIVOS... NO FALTAN
        self.mongo = pymongo.MongoClient()
        self.db = self.mongo[self.db_name]
        self.coll = self.db["episodes"]
        self.n_episodes_mongo = self.coll.count()
        rospy.loginfo(" - Episodes in DB: " + str(self.n_episodes_mongo))
        

    def setup_files(self):
        self.deltas.parse_files()
        self.fulls.parse_files()

        # full as delta numbers
        max_full = 0
        max_full_eps = 0
        if self.fulls.found:
            max_full = max(self.fulls.found)
            max_full = min(max_full, self.fulls.to_n(self.max_episodes))
            max_full_eps = self.fulls.to_eps(max_full)
            # print "Max FULL: " + str(max_full)
        max_full_as_deltas_n = self.deltas.to_n(max_full_eps)
        full_as_deltas = set(range(1, max_full_as_deltas_n + 1))

        # get missing deltas which are greater than max full eps.
        missing_deltas = set()
        gt_deltas = self.deltas.found - full_as_deltas
        if gt_deltas:
            max_delta = max(gt_deltas)
            max_delta = min(max_delta, self.deltas.to_n(self.max_episodes))
            desired_deltas = set(range(max_full_as_deltas_n + 1, max_delta + 1))
            missing_deltas = desired_deltas - gt_deltas

        # minimum valid episodes
        min_valid_delta = 0
        if missing_deltas:
            min_valid_delta = min(missing_deltas) - 1
        elif gt_deltas:
            max_delta = max(gt_deltas)
            max_delta = min(max_delta, self.deltas.to_n(self.max_episodes))
            min_valid_delta = max_delta
        min_valid_delta_eps = self.deltas.to_eps(min_valid_delta)
        min_valid_episodes = max(min_valid_delta_eps, max_full_eps)

        gt_deltas_valid = set()
        for dt in gt_deltas:
            if dt <= min_valid_delta:
                gt_deltas_valid.add(dt)
        
        # print "Max FULL EPS: " + str(max_full_eps)
        # print "Interesting deltas: " + str(gt_deltas)
        # print "Desired Deltas: " + str(desired_deltas)
        # print "Missing deltas: " + str(missing_deltas)
        # print "Min valid eps: " + str(min_valid_episodes)
        return min_valid_episodes, max_full, gt_deltas_valid

    def unzip(self, filename):
        if not os.path.exists(filename):
            rospy.logwarn(" - Cannot perform unzip file. Required file not found: " + filename)
            return False

        with tarfile.open(filename, "r:gz") as tar:
            tar.extractall(self.gen_path)
        return True

    def gzip(self, directory):
        # rospy.loginfo("   ... compressing directory: " + directory)
        if not os.path.isdir(directory):
            rospy.logwarn(" - Cannot perform compression. Required directory not found: " + directory)
            return
        zipname = directory + ".tar.gz"

        # remove gzip
        if os.path.exists(zipname):
            os.remove(zipname)

        # create tar.gz
        with tarfile.open(zipname, "w:gz") as tar:
            tar.add(directory, arcname=os.path.basename(directory))

        # remove directory
        shutil.rmtree(directory)
        # rospy.loginfo("   ... compression finished.")

    def delta_dump_dummy(self, path):
        os.makedirs(path)
        open(path + "/" + "a.txt", 'a').close()
        open(path + "/" + "b.txt", 'a').close()
        open(path + "/" + "c.txt", 'a').close()

    def record(self, record_name, start, end):
        record_path = self.gen_path + "/" + record_name

        # remove old record
        if os.path.isdir(record_path):
            shutil.rmtree(record_path)

        # generate record
        query = "{uid: {$gte: %d, $lte: %d}}" % (start, end)
        command = "mongodump --quiet --db %s --out %s --query '%s'" % (self.db_name, record_path, query)
        os.system(command)

        # compression        
        self.gzip(record_path)

    def load(self, record_name):

        # UNZIP
        zipname = self.gen_path + "/" + record_name + ".tar.gz"
        if not self.unzip(zipname):
            return False

        # LOAD
        unzip_dir = self.gen_path + "/" + record_name
        unzip_dir_in = unzip_dir + "/" + self.db_name
        command = "mongorestore --quiet --db %s --dir %s" % (self.db_name, unzip_dir_in)
        os.system(command)

        # remove unzipped dir
        if os.path.isdir(unzip_dir):
            shutil.rmtree(unzip_dir)
        return True

    def prepare(self):
        # Empezar contador en multiplo de 10.
        n_recorded_eps, max_full, gt_deltas_valid = self.setup_files()
        init_episodes = 10 * int(self.n_episodes_mongo / 10)
        if self.n_episodes_mongo < n_recorded_eps:
            full_eps = self.fulls.to_eps(max_full)
            if self.n_episodes_mongo < full_eps:
                full_record_name = self.fulls.gen_name(full_eps)
                rospy.logwarn("Regenerating DB episodes from FULL record: " + full_record_name)
                self.load(full_record_name)

            # ONLY LOAD REQUIRED EPS
            # print "Delta records to load: " + str(gt_deltas_valid)
            dts = list(gt_deltas_valid)
            dts.sort()
            counter = full_eps
            for m in dts:
                counter += self.deltas.chunk_size
                if self.n_episodes_mongo > counter:
                    continue
                eps = self.deltas.to_eps(m)
                record_name = self.deltas.gen_name(eps)
                rospy.logwarn("Regenerating DB episodes from DELTA record: " + record_name)
                self.load(record_name)

            init_episodes = n_recorded_eps
        
        # REBUILD MISSING RECORDS
        missing_deltas = self.deltas.compute_missing(init_episodes)
        missing_fulls = self.fulls.compute_missing(init_episodes)
        rospy.loginfo("Missing Full records: " + str(missing_fulls))
        for m in missing_fulls:
            eps = self.fulls.to_eps(m)
            dump_name = self.fulls.gen_name(eps)
            rospy.loginfo("Regenerating record %s from database." % dump_name)
            self.record(dump_name, 0, eps)

        rospy.loginfo("Missing Delta Records: " + str(missing_deltas))
        for m in missing_deltas:
            eps = self.deltas.to_eps(m)
            dump_name = self.deltas.gen_name(eps)
            rospy.loginfo("Regenerating record %s from database." % dump_name)
            self.record(dump_name, eps - self.deltas.chunk_size, eps)

        return init_episodes

    def execute(self):
        episodes = self.prepare()
        rospy.loginfo(" - Starting EPISODES: " + str(episodes))

        while not rospy.is_shutdown() and episodes < self.max_episodes:

            episodes = self.gen_tree(episodes)

            # record deltas
            if self.deltas.is_checkpoint(episodes):
                dump_name = self.deltas.gen_name(episodes)
                rospy.loginfo("  -  " + str(episodes) + " episodes. Recording delta: '" + dump_name + "'")
                self.record(dump_name, episodes-self.deltas.chunk_size, episodes)

            # record full dumps
            if self.fulls.is_checkpoint(episodes):
                dump_name = self.fulls.gen_name(episodes)
                rospy.loginfo(" === " + str(episodes) + " episodes === Recording complete DB: '" + dump_name + "'")
                self.record(dump_name, 0, episodes)

                rospy.loginfo(" .......................................................... ")

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
        try:
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
        except:
            pass

        return ep10.uid


def main():
    try:
        rospy.init_node("ltm_benchmark_db_gen")
        BenchmarkDBGenerator().execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
