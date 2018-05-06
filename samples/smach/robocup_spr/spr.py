#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

# machines
import crowd_analysis
import riddle_game
import blind_man_game
import ltm.smach as ltm


def get_instance():
    sm = smach.StateMachine(outcomes=['succeeded'])
    ltm.register_state(sm, ["robocup", "stage_1", "spr"])
    with sm:

        smach.StateMachine.add(
            'CROWD_ANALYSIS',
            crowd_analysis.get_instance(),
            transitions={'succeeded': 'RIDDLE_GAME'}
        )

        smach.StateMachine.add(
            'RIDDLE_GAME',
            riddle_game.get_instance(),
            transitions={'succeeded': 'BLIND_MAN_GAME'}
        )

        smach.StateMachine.add(
            'BLIND_MAN_GAME',
            blind_man_game.get_instance(),
            transitions={'succeeded': 'succeeded'}
        )
    return sm


def main():
    try:
        rospy.init_node('robocup_spr', log_level=rospy.DEBUG)

        # build machine
        sm = get_instance()
        ltm.setup(sm)

        # smach introspection server
        sis = smach_ros.IntrospectionServer('robocup_spr_sis', sm, '/SM_SPR')
        sis.start()

        # execute machine
        sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
