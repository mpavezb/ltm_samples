#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from util import main, get_sub_machine
import ltm_addons.smach as ltm

"""
Tree Structure
---------------------------------
          (root)
      /   |   |   |
   (A1) (A2) {A3} (A4)
    
( ): registered
{ }: unregistered 
"""


def get_instance():
    sm = get_sub_machine(["A1", "A2", "", "A4"])
    ltm.register_state(sm, ["root"])
    return sm


if __name__ == '__main__':
    rospy.init_node('test_missing_child', log_level=rospy.DEBUG)
    main(get_instance())
