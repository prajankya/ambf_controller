#!/usr/bin/python
import os
import sys

import yaml
import rospkg

from logger import logger as log
from kin_tree import Body, Joint
# =============================================================================== Initializations


def readYaml(yaml_file):

    # stream = open(yaml_file, 'r')
    # dictionary = yaml.load(stream)
    # for key, value in dictionary.items():
    #     print (key + " : " + str(value))

    with open(yaml_file, 'rb') as f:
        data = yaml.load(f)
        # log.debug(data['bodies'])
        bodies_namelist = data['bodies']
        bodies = []
        for body_name in bodies_namelist:
            b = Body(data[body_name])
            log.debug(b)
