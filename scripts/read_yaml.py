#!/usr/bin/python
import os
import sys

import yaml
import rospkg

from logger import logger as log

# =============================================================================== Initializations
# can be used as default file
#
# rospack = rospkg.RosPack()
# DEFAULT_YAML_FILEPATH = os.path.join(rospack.get_path(
#     'ambf_controller'), "example_robots", "blender-kuka.yaml")


def readYaml(yaml_file):

    # stream = open(yaml_file, 'r')
    # dictionary = yaml.load(stream)
    # for key, value in dictionary.items():
    #     print (key + " : " + str(value))

    with open(yaml_file, 'rb') as f:
        data = yaml.load(f)
        log.debug(data)
