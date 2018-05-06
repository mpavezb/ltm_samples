#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from util import main, get_sub_machine
import ltm.smach as ltm

"""
Tree Structure
---------------------------------
               (root) _
             /         |
          (A)         (B)
      /   |   |       /  |
   (A1) (A2) (A3)  {B1} {B2}
    
( ): registered
{ }: unregistered 
"""


def get_instance():

    root = smach.StateMachine(outcomes=['ok'])
    ltm.register_state(root, ["root"])

    with root:
        node_1 = get_sub_machine(['A1', 'A2', 'A3'])
        node_2 = get_sub_machine(['', ''])
        ltm.register_state(node_1, ['A'])
        ltm.register_state(node_2, ['B'])
        smach.StateMachine.add('A', node_1, transitions={'ok': 'B'})
        smach.StateMachine.add('B', node_2, transitions={'ok': 'ok'})

    return root


if __name__ == '__main__':
    rospy.init_node('test_no_children', log_level=rospy.DEBUG)
    main(get_instance())
