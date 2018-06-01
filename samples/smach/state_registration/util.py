#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import ltm_addons.smach as ltm


class Dummy(smach.State):
    def __init__(self, name):
        smach.State.__init__(self, outcomes=['ok'])
        self.name = name

    def execute(self, userdata):
        rospy.logwarn('[state]| Dummy |: ' + self.name)
        rospy.sleep(0.5)
        return 'ok'


class Looper(smach.State):

    def __init__(self, name, times):
        smach.State.__init__(self, outcomes=['ok', 'loop'])
        self.name = name
        self.times = times
        self.count = 0

    def execute(self, userdata):
        self.count += 1
        rospy.logwarn('[state]| Lopper |: ' + self.name + " execution " + str(self.count) + "/" + str(self.times))
        rospy.sleep(0.5)
        if self.count < self.times:
            return 'loop'
        return 'ok'


def get_sub_machine(children_tags):
    """
    returns a smach.State machine with len(children_tags) children. Only children whose tag is not "" are registered.
    """
    node = smach.StateMachine(outcomes=['ok'])

    with node:

        for idx, tag in enumerate(children_tags):
            register = tag != ""

            name = tag
            if not register:
                name = "<Unnamed " + str(idx+1) + ">"

            leaf = Dummy(name)

            if register:
                ltm.register_state(leaf, [tag])

            transition = 'ok'
            if idx + 1 < len(children_tags):
                transition = children_tags[idx + 1].upper()
                if transition == "":
                    transition = ("<Unnamed " + str(idx+2) + ">").upper()

            smach.StateMachine.add(name.upper(), leaf, transitions={'ok': transition})

    return node


def main(sm):
    try:
        # setup machine
        ltm.setup(sm)

        # execute machine
        ud = smach.UserData()
        sm.execute(ud)

    except rospy.ROSInterruptException:
        pass