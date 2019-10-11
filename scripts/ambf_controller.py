#!/usr/bin/python

import click

from colorama import Fore, Back, Style, init

from solver import Solver
from solver import _SolverCollection
from read_yaml import readYaml
from ambf_funcs import connect_to_ambf_client

from logger import logger as log

# =============================================================================== CLI
@click.command()
@click.option('-r', '--robot_file', 'yaml_file', help='Robot Yaml File path', type=click.Path(exists=True), required=False)
# @click.option('-d', '--verbose', 'is_verbose', default=True, is_flag=True, help="Will print verbose messages.")
def controller(yaml_file):
    # =============================================================================== Initializations
    global IS_STANDALONE

    if yaml_file != None:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from : " +
                  Fore.RESET+Style.RESET_ALL+yaml_file)
        IS_STANDALONE = True
    else:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from AMBF Simulator." +
                  Fore.RESET+Style.RESET_ALL)
        IS_STANDALONE = False

    # =============================================================================== Load all solvers
    solversCollection = _SolverCollection('solvers')

    solvers = solversCollection.getAllSolvers()
    log.debug("Kinematics Solvers detected :")
    log.debug(solvers)
    sol = solvers['DELTA']()
    sol.FK("", "")
    # =============================================================================== Load Map
    # Getting Parent-child map
    if IS_STANDALONE:
        readYaml(yaml_file)  # TODO : Yet to write the code
    else:
        connect_to_ambf_client()

    #


if __name__ == '__main__':
    controller()
