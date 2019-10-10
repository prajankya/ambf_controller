#!/usr/bin/python

import importlib
import pkgutil
import os
import sys
import logging

import click
import rospkg

from ambf_client import Client  # Import the Client from ambf_client package
from colorama import Fore, Back, Style, init

from ambf_solver import Solver
from ambf_solver import SolverCollection

# =============================================================================== Initializations
rospack = rospkg.RosPack()
YAML_FILEPATH = os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "blender-kuka.yaml")

# create logger
logger = logging.getLogger(Fore.MAGENTA + 'AMBF_ctrl')

# =============================================================================== CLI
@click.command()
@click.option('-r', '--robot_file', 'yaml_file', default=YAML_FILEPATH, help='Robot Yaml File path', type=click.Path(exists=True))
@click.option('-d', '--verbose', 'is_verbose', default=True, is_flag=True, help="Will print verbose messages.")
def controller(yaml_file, is_verbose):
    # =============================================================================== Initializations
    if is_verbose:
        logger.setLevel(logging.DEBUG)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    if is_verbose:
        ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - "+Fore.YELLOW+"%(levelname)s"+Fore.RESET+" - %(message)s")

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

    logger.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from : " +
                 Fore.RESET+Style.RESET_ALL+yaml_file)

    connect_ambf_client()


def connect_ambf_client():
    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bi-directional communication
    _client.connect()

    # check if AMBF is running by checking number of objects loaded in AMBF(using rostopic inside)
    if(len(_client.get_obj_names()) == 0):
        sys.exit(Fore.RED+"AMBF Simulator is not running")

    # Printing summary of ambf_client node
    logger.debug(_client.print_summary())
    logger.debug('\n\n----')
    logger.debug("List of Objects")
    logger.debug(_client.get_obj_names())


if __name__ == '__main__':
    # controller()
        # for(name) in pkgutil.iter_modules(["solvers"]):
    #     print(name)
    #     importlib.import_module('.' + name, __package__)
    # print(ambf_solver)
    my_solvers = SolverCollection('solvers')
    # all_my_base_classes = {
    #     cls.__name__: cls for cls in Solver.__subclasses__()}

    # print(all_my_base_classes)

    # all_my_base_classes = {
    #     cls.__name__: cls for cls in solvers._MyBase.__subclasses__()}

    # my_import('solvers')
    # __import__('solvers')


def my_import(name):
    components = name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod
