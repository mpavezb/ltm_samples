#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
import rospy
from fake_person_entity import PersonEntityFaker
from fake_object_entity import ObjectEntityFaker
import ltm_samples.msg


class EntityManager(object):

    def __init__(self, namespace, faker, msg_class):
        self.faker = faker
        self.registry = dict()
        self.ns = namespace
        self.max_instances = rospy.get_param("~" + namespace + "/max_instances", 5)
        self.update_period = rospy.get_param("~" + namespace + "/update_period", 1.0)
        self.publisher = rospy.Publisher("~" + namespace + "/updates", msg_class, queue_size=10)
        self.initial_f = rospy.get_param("~" + namespace + "/initial_fields", [])
        self.static_f = rospy.get_param("~" + namespace + "/static_fields", [])
        self.dynamic_f = rospy.get_param("~" + namespace + "/dynamic_fields", [])
        self.ignore_f = rospy.get_param("~" + namespace + "/ignore_fields", [])
        rospy.Timer(rospy.Duration(self.update_period), self.timer_callback)

    def get_remaining_static(self, msg):
        null = self.faker.null()
        remaining = []
        for field in self.static_f:
            value = msg.__getattribute__(field)
            null_value = null.__getattribute__(field)
            if value == null_value:
                remaining.append(field)
        return remaining

    def add_entity(self, uid):
        generated = self.faker.generate()
        msg = self.faker.null()
        msg.uid = uid
        log_str = "["
        for field in self.initial_f:
            if field == "uid":
                continue
            value = generated.__getattribute__(field)
            if isinstance(value, (basestring, int, float)):
                log_str = log_str + str(value) + ", "
            msg.__setattr__(field, value)
        log_str = log_str[:-2] + "]"
        self.registry[uid] = msg
        self.publisher.publish(self.faker.normalize(msg))
        rospy.loginfo("{" + self.ns + "}: new entity (" + str(uid) + "): " + log_str)

    def add_static_field(self, msg, remaining):
        # get field and remove from remaining
        pos = random.randint(0, len(remaining)-1)
        field = remaining.pop(pos)
        generated = self.faker.generate()
        value = generated.__getattribute__(field)

        # update and publish
        msg.__setattr__(field, value)
        self.registry[msg.uid] = msg
        self.publisher.publish(self.faker.normalize(msg))

        # LOGGING
        log_str = ""
        if isinstance(value, (basestring, int, float)):
            log_str = log_str + "=" + str(value)
        rospy.loginfo("{" + self.ns + "}: add static field (" + str(msg.uid) + "): " + field + log_str)
        if not remaining:
            rospy.loginfo("{" + self.ns + "}: All static fields are set for (" + str(msg.uid) + ").")

    def generate_dynamic_field(self, msg):
        # get field and remove from remaining
        pos = random.randint(0, len(self.dynamic_f)-1)
        field = self.dynamic_f[pos]
        generated = self.faker.generate()
        value = generated.__getattribute__(field)

        was_null = self.faker.is_field_null(msg, field)

        # update and publish
        msg.__setattr__(field, value)
        self.registry[msg.uid] = msg
        self.publisher.publish(self.faker.normalize(msg))

        # LOGGING
        log_str = ""
        if isinstance(value, (basestring, int, float)):
            log_str = log_str + "=" + str(value)

        if was_null:
            rospy.loginfo("{" + self.ns + "}: add dynamic field (" + str(msg.uid) + "): " + field + log_str)
        else:
            rospy.loginfo("{" + self.ns + "}: modify dynamic field (" + str(msg.uid) + "): " + field + log_str)

    def timer_callback(self, event):
        # modify this entity
        uid = random.randint(1, self.max_instances)

        # new value: only set initial fields
        if uid not in self.registry:
            self.add_entity(uid)
            return

        # choose between static and dynamic (1 field at a time)
        msg = self.registry[uid]

        static_remaining = self.get_remaining_static(msg)
        do_static = static_remaining and random.randint(0, 1) == 0
        if do_static:
            self.add_static_field(msg, static_remaining)
            return

        self.generate_dynamic_field(msg)


class FakeShortTermMemory(object):

    def __init__(self):

        self.entities = rospy.get_param("~entities", [])

        # person entity
        if "person" in self.entities:
            faker = PersonEntityFaker()
            self.person_manager = EntityManager("person", faker, ltm_samples.msg.PersonEntity)

        # object entity
        if "object" in self.entities:
            faker = ObjectEntityFaker()
            self.object_manager = EntityManager("object", faker, ltm_samples.msg.ObjectEntity)

        rospy.loginfo("The Short Term Memory is ready to generate false information.")
        rospy.loginfo("Considered entities: " + str(self.entities))


def main():
    rospy.init_node('ltm_samples_short_term_memory')
    FakeShortTermMemory()

    # Wait for ctrl-c to stop the application
    rospy.on_shutdown(lambda: rospy.logwarn("Ha det bra!."))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
