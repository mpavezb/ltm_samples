#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from util import main, Dummy, Looper
import ltm_addons.smach as ltm

"""
Tree Structure
---------------------------------
             (root)
    /  |   |   |   |   |   |   |
 (A1)(A2)(A3)(A2)(A3)(A2)(A3)(A4)
    
( ): registered
{ }: unregistered 
"""


def get_instance():

    sm = smach.StateMachine(outcomes=['ok'])
    ltm.register_state(sm, ["root"])
    with sm:

        state_1 = Dummy("A1")
        state_2 = Dummy("A2")
        state_3 = Looper("A3", times=3)
        state_4 = Dummy("A4")

        # register children
        ltm.register_state(state_1, ["A1"])
        ltm.register_state(state_2, ["A2"])
        ltm.register_state(state_3, ["A3"])
        ltm.register_state(state_4, ["A4"])

        smach.StateMachine.add('A1', state_1, transitions={'ok': 'A2'})
        smach.StateMachine.add('A2', state_2, transitions={'ok': 'A3'})
        smach.StateMachine.add('A3', state_3, transitions={'ok': 'A4', 'loop': 'A2'})
        smach.StateMachine.add('A4', state_4, transitions={'ok': 'ok'})
    return sm


if __name__ == '__main__':
    rospy.init_node('test_loop', log_level=rospy.DEBUG)
    main(get_instance())
