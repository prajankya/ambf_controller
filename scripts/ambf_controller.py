#!/usr/bin/python

import os
import rospkg

import yaml
import click
from colorama import Fore, Back, Style, init

from solver import Solver
from kin_tree import Chain
from logger import logger as log
from solver import _SolverCollection
from ambf_funcs import connect_to_ambf_client


rospack = rospkg.RosPack()

# =============================================================================== CLI
@click.command()
@click.option('-r', '--robot_file', 'yaml_file', default=os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "blender-kuka.yaml"), help='Robot Yaml File path', type=click.Path(exists=True), required=False)
@click.option('-y', '--yaml', 'IS_STANDALONE', default=True, is_flag=True, help="Load Robot from Yaml file instead of AMBF_client.")
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

    # =============================================================================== Loading chain
    # Getting Kinematics chain
    chain = None
    solver_to_use = None
    if IS_STANDALONE:
        with open(yaml_file, 'rb') as f:
            data = yaml.load(f)
            chain = Chain(data)
            solver_to_use = data['solver']
    else:
        connect_to_ambf_client()   # Yet to be implemented from AMBF client side
        # chain = getChainFromAMBFClient(); #:yet to be implemented
        # solver_to_use  = getSolverForTheChain()# yet to be implemented

    log.debug("Chain :")
    log.debug(chain)

    log.debug("Solver to use :")
    log.debug(solver_to_use)


if __name__ == '__main__':
    controller()
