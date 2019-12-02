#!/usr/bin/env python
import time

import rospy
import numpy as np
from geometry_msgs.msg import Pose


points = np.array([[0.0, -0.4, 0.55], [0.5, 0.0, 0.4], [-0.6, 0.7, 0.5]])

if __name__ == "__main__":
    rospy.init_node('trajectory')
    pub = rospy.Publisher("/ambf/setPose", Pose, queue_size=1)

    ind = 1
    while ind < len(points):
        intermediate_points_x = np.linspace(points[ind - 1][0], points[ind][0], 20)  # Break into 20 parts
        intermediate_points_y = np.linspace(points[ind - 1][1], points[ind][1], 20)  # Break into 20 parts
        intermediate_points_z = np.linspace(points[ind - 1][2], points[ind][2], 20)  # Break into 20 parts

        rate = rospy.Rate(5) # 5 Hz
        for i in range(len(intermediate_points_x)):
            pose = Pose()
            pose.position.x = intermediate_points_x[i]
            pose.position.y = intermediate_points_y[i]
            pose.position.z = intermediate_points_z[i]

            print(pose)
            pub.publish(pose)
            rate.sleep()

        ind += 1
