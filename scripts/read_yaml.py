#!/usr/bin/python
import os
import sys

import yaml
import rospkg
from ambf_client import Client  # Import the Client from ambf_client package


# ----------------------------------------------------------------------- Initializations
# Use rospack to get absolute path of file
# rospack = rospkg.RosPack()
# YAML_FILEPATH = os.path.join(rospack.get_path(
#     'ambf_controller'), "example_robots", "blender-kuka.yaml")

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

# ----------------------------------------------------------------------- Body

# check if AMBF is running by checking number of objects loaded in AMBF(using rostopic inside)
if(len(_client.get_obj_names()) == 0):
    sys.exit("AMBF Simulator is not running")


# Printing summary of ambf_client node
print(_client.print_summary())
print('\n\n----')
print("List of Objects")
print(_client.get_obj_names())

# stream = open(YAML_FILEPATH, 'r')
# dictionary = yaml.load(stream)
# for key, value in dictionary.items():
#     print (key + " : " + str(value))

# with open(YAML_FILEPATH, 'rb') as f:
#     data = yaml.load(f)
#     print(data)
