#! /usr/bin/env python
import sys
import rospy
# Import the Client from ambf_client package
from ambf_client import Client

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

_handle = _client.get_obj_handle('base')

_tip = _client.get_obj_handle('link7')

print('\n\n----')


rate = rospy.Rate(10)
while not rospy.is_shutdown():
    position = _tip.get_pos()
    string = str(rospy.get_time())+"\t x= " + str(position.x) + ",\t y= " + str(position.y) + ",\t z= " + str(position.z)
    sys.stdout.write('\r' + string)
    sys.stdout.flush() # important

    rate.sleep()

print('shutdown complete')
