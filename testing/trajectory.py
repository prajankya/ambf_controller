#!/usr/bin/env python
import time

import rospy
import numpy as np
from geometry_msgs.msg import Pose

points = np.array([[0.6, -0.3, 0.55], [0.2, 0.0, 0.4],[-0.6,-0.3, 0.6]])


if __name__ == "__main__":
    rospy.init_node('trajectory')
    pub = rospy.Publisher("/ambf/setPose", Pose, queue_size=1)

    print("Starting")
    ind = 1
    while ind < len(points):
        intermediate_points_x = np.linspace(
            points[ind - 1][0], points[ind][0], 100)  # Break into 50 parts
        intermediate_points_y = np.linspace(
            points[ind - 1][1], points[ind][1], 100)  # Break into 50 parts
        intermediate_points_z = np.linspace(
            points[ind - 1][2], points[ind][2], 100)  # Break into 50 parts

        rate = rospy.Rate(10)
        for i in range(len(intermediate_points_x)):
            pose = Pose()
            pose.position.x = intermediate_points_x[i]
            pose.position.y = intermediate_points_y[i]
            pose.position.z = intermediate_points_z[i]

            print(pose)
            pub.publish(pose)
            rate.sleep()

        ind += 1
