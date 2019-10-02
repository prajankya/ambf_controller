#!/usr/bin/python

import os

import click
import rospkg

from colorama import Fore, Back, Style, init

# =============================================================================== Initializations
init()

rospack = rospkg.RosPack()
YAML_FILEPATH = os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "blender-kuka.yaml")


# =============================================================================== CLI
@click.command()
@click.option('-r', '--robot_file', 'yaml_file', default=YAML_FILEPATH, help='Robot Yaml File path', type=click.Path(exists=True))
# @click.argument('filename', type=click.Path(exists=True))
def controller(yaml_file):

    print Style.BRIGHT+Fore.BLUE+"Loading Robot from : " + \
        Fore.RESET+Style.RESET_ALL+yaml_file


if __name__ == '__main__':
    controller()
