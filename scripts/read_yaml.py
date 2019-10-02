#!/usr/bin/python
import yaml
import os
import rospkg

# Use rospack to get absolute path of file
rospack = rospkg.RosPack()
YAML_FILEPATH = os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "blender-kuka.yaml")

stream = open(YAML_FILEPATH, 'r')
dictionary = yaml.load(stream)
for key, value in dictionary.items():
    print (key + " : " + str(value))

# with open(YAML_FILEPATH, 'rb') as f:
#     data = yaml.load(f)
#     print(data)
