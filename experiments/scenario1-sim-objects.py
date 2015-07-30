#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import itertools
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('sim_object_spawner')

    W = client.SimWorld()

    person = W.add_object("person", "tim", -6, 0, 0)

    path = []
    for i in range(0, 1000):
        path += [(-6, 0, 0), (6, 0, 0), (6, -7, 0), (-6, -7, 0)]

    person.set_path(path=path, path_vel=2.0)
