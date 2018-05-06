#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from util import Dummy, main

"""
Tree Structure
---------------------------------
               
           (state)
    
( ): registered
{ }: unregistered 
"""


def get_instance():
    return Dummy('root')


if __name__ == '__main__':
    rospy.init_node('test_single_state', log_level=rospy.DEBUG)
    main(get_instance())
