#!/usr/bin/python

import os
import rospkg

import click
from colorama import Fore, Back, Style, init

from solver import Solver
from solver import _SolverCollection
from read_yaml import readYaml
from ambf_funcs import connect_to_ambf_client

from logger import logger as log

rospack = rospkg.RosPack()

# =============================================================================== CLI
@click.command()
@click.option('-r', '--robot_file', 'yaml_file', default=os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "blender-kuka.yaml"), help='Robot Yaml File path', type=click.Path(exists=True), required=False)
@click.option('-y', '--yaml', 'IS_STANDALONE', default=False, is_flag=True, help="Load Robot from Yaml file instead of AMBF_client.")
def controller(yaml_file, IS_STANDALONE):
    # =============================================================================== Initializations
    if IS_STANDALONE:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from : " +
                  Fore.RESET+Style.RESET_ALL+yaml_file)
    else:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from AMBF Simulator." +
                  Fore.RESET+Style.RESET_ALL)

    # =============================================================================== Load all solvers
    solversCollection = _SolverCollection('solvers')

    solvers = solversCollection.getAllSolvers()
    log.debug("Kinematics Solvers detected :")
    log.debug(solvers)

    # =============================================================================== Load Map
    # Getting Parent-child map
    if IS_STANDALONE:
        readYaml(yaml_file)
    else:
        connect_to_ambf_client()   # TODO : Yet to write the code to get kintree


if __name__ == '__main__':
    controller()
