#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 20:51:56 2021

"""

import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

if __name__ == '__main__':

    rospy.init_node('rvizMarker_simulation', anonymous=False)
    visualPublisher1 = rospy.Publisher(
        "contour", Marker, queue_size=10)

    while not rospy.is_shutdown():
        try:

            contourMarker = Marker()
            contourMarker.color = ColorRGBA(0.70833, 0.59722, 0, 1)
            contourMarker.ns = 'obstacle'
            contourMarker.action = Marker.ADD
            contourMarker.header.frame_id = 'world'
            contourMarker.header.stamp = rospy.Time.now()

            contourMarker.type = 3
            contourMarker.id = 1

            contourMarker.pose.position.x = 0.5  # -0.5
            contourMarker.pose.position.y = 0.1  # -0.6
            contourMarker.pose.position.z = 0.04

            contourMarker.scale.x = 0.19
            contourMarker.scale.y = 0.19
            contourMarker.scale.z = 0.08  # height

            visualPublisher1.publish(contourMarker)

        except rospy.ROSException as e:
            print("ROSException: %s" % e)
