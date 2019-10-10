#!/usr/bin/python
import os
import sys

import yaml
import rospkg


def readYaml(yaml_file, logger):
    # stream = open(yaml_file, 'r')
    # dictionary = yaml.load(stream)
    # for key, value in dictionary.items():
    #     print (key + " : " + str(value))

    with open(yaml_file, 'rb') as f:
        data = yaml.load(f)
        logger.debug(data)
