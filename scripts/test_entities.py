#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospkg
import rospy

from ltm.srv import *
from ltm_samples.msg import PersonEntity
from ltm_samples.srv import PersonEntitySrv, PersonEntitySrvRequest


class EntityInterface(object):

    def __init__(self):
        self.get_client = rospy.ServiceProxy('/robot/ltm/entity/people/get', PersonEntitySrv)
        self.query_client = rospy.ServiceProxy('/robot/ltm/db/query', QueryServer)

    def setup(self):
        self.get_client.wait_for_service()
        self.query_client.wait_for_service()

    def get(self, uids, stamps=None):
        req = PersonEntitySrvRequest()
        req.uids = uids
        if stamps:
            req.stamps = stamps
        resp = self.get_client(req)
        return resp

    def look_for(self, uid, stamp=None):
        stamps = None
        if stamp:
            stamps = [stamp]
        resp = self.get([uid], stamps)
        if len(resp.msgs) is not 1:
            print "ERROR: Person with uid: " + str(uid) + " not found."
            person = PersonEntity()
        else:
            person = resp.msgs[0]
            person.face = None
            person.body = None
        return person

    def query(self, json):
        query = QueryServerRequest()
        query.target = "entity"
        query.semantic_type = "people"
        query.json = json
        resp = self.query_client(query)
        return resp

    def query_log(self, json):
        query = QueryServerRequest()
        query.target = "entity_trail"
        query.semantic_type = "people"
        query.json = json
        resp = self.query_client(query)
        print resp
        return resp.entities[0].uids, resp.entities_trail[0].uids


def show(name, uid, secs, nsecs, fields=[]):
    it = EntityInterface()
    t = rospy.Time(secs, nsecs)
    person = it.look_for(uid, t)

    print("======================================================================")
    print(name + ", stamp: " + str(secs) + ", log_uid:" + str(person.meta.log_uid) + ", person: " + person.name + " " + person.last_name)    
    print("----------------------------------------------------------------------")
    if not fields:
        print(person)
    else:
        for f in fields:
            print(f + ": " + str(getattr(person, f)))
    print("")


if __name__ == '__main__':
    rospy.init_node('test_ltm_entities')

    it = EntityInterface()
    it.setup()

    # ALL PEOPLE
    it.query('{}')

    static_f = ['name', 'last_name', 'genre', 'age', 'country', 'city', 'birthday']

    # show("COMPLETE", 2, 0, 0)
    show("INIT MSG", 2, 1540726254, 0, static_f)
    show("STATIC_0", 2, 1540726254, 500000000, static_f) # name      Caleb, Mendoza
    show("STATIC_1", 2, 1540726256, 500000000, static_f) # city      Lake Jennifer 
    show("STATIC_2", 2, 1540726258, 500000000, static_f) # country   Uzbekistan    
    show("STATIC_3", 2, 1540726266, 500000000, static_f) # age       9             
    show("STATIC_4", 2, 1540726270, 500000000, static_f) # birthday  --            
    show("STATIC_5", 2, 1540726278, 500000000, static_f) # genre     0             
    show("LAST MSG", 2, 1540726895, 500000000, static_f)


    # it.query_log('{$query: {entity_uid: 2, timestamp: { $lte: 1540684542 }}, $orderby: { timestamp: -1}}')

