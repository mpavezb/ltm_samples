#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import ltm.smach as ltm


class Talk(smach.State):
    def __init__(self, text=""):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.text = text

    def execute(self, userdata):
        rospy.logwarn('[state]| Talk |: ' + self.text)
        rospy.sleep(0.5)
        return 'succeeded'


class LookForPeople(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        ltm.register_state(self, ["look_for_crowd"])

    def execute(self, userdata):
        rospy.logwarn('[state]| Look for people |: I am looking for people. Where are you hiding?.')
        rospy.sleep(1.0)
        return 'succeeded'


class AnalyzePeople(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        ltm.register_state(self, ["analyze_crowd"])
        self.failed = False

    def execute(self, userdata):
        rospy.logwarn('[state]| Analyze people |: mmm ... <beep> i am gathering information about you <bop>.')
        rospy.sleep(1.0)
        if not self.failed:
            self.failed = True
            rospy.logwarn('[state]| Analyze people |: mmm ... <KABOOM!> i have failed. <UPSS>.')
            return 'failed'
        return 'succeeded'
