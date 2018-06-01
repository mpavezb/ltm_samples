#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from util import Dummy, main
import ltm_addons.smach as ltm

"""
Tree Structure
---------------------------------
               {root} 
               /     |
           (LEAF_1) (LEAF_2)
    
( ): registered
{ }: unregistered 
"""


def get_instance():
    sm = smach.StateMachine(outcomes=['ok'])
    with sm:

        leaf_1 = Dummy("leaf_1")
        leaf_2 = Dummy("leaf_2")

        # register children
        ltm.register_state(leaf_1, ["leaf_1"])
        ltm.register_state(leaf_2, ["leaf_2"])

        smach.StateMachine.add('LEAF_1', leaf_1, transitions={'ok': 'LEAF_2'})
        smach.StateMachine.add('LEAF_2', leaf_2, transitions={'ok': 'ok'})
    return sm


if __name__ == '__main__':
    rospy.init_node('test_no_root', log_level=rospy.DEBUG)
    main(get_instance())
