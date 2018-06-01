#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from util import get_sub_machine, main
import ltm_addons.smach as ltm

"""
Tree Structure
---------------------------------
               (root) _
             /         |
       {GRANDPA}      (B)
         /           /   |
     {FATHER}      (B1) (B2)
    /   |   |
 (A1) (A2) (A3)
 
( ): registered
{ }: unregistered 
"""


def get_instance():

    root = smach.StateMachine(outcomes=['ok'])
    ltm.register_state(root, ["root"])

    grandpa = smach.StateMachine(outcomes=['ok'])
    with grandpa:
        father = get_sub_machine(['A1', 'A2', 'A3'])
        smach.StateMachine.add('FATHER', father, transitions={'ok': 'ok'})

    with root:
        node_b = get_sub_machine(['B1', 'B2'])
        ltm.register_state(node_b, ['node_b'])
        smach.StateMachine.add('GRANDPA', grandpa, transitions={'ok': 'B'})
        smach.StateMachine.add('B', node_b, transitions={'ok': 'ok'})

    return root


if __name__ == '__main__':
    rospy.init_node('test_no_parents', log_level=rospy.DEBUG)
    main(get_instance())
