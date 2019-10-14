#!/usr/bin/python
import os
import sys

import yaml
import rospkg

from logger import logger as log
from kin_tree import Chain, Body, Joint
# =============================================================================== Initializations


def readYaml(yaml_file):

    # stream = open(yaml_file, 'r')
    # dictionary = yaml.load(stream)
    # for key, value in dictionary.items():
    #     print (key + " : " + str(value))

    with open(yaml_file, 'rb') as f:
        data = yaml.load(f)

        bodies = {}
        for body_name in data['bodies']:
            bodies[data[body_name]['name']] = Body(data[body_name])

        joints = {}
        for joint_name in data['joints']:
            joints[data[joint_name]['name']] = Joint(data[joint_name])

        # name can be implemented in YAML
        chain = Chain(bodies, joints)
        log.debug(chain.getJoint(chain.getBody(
            chain.getBody("base").children[0]).child_joints[0]))
