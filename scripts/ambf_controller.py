#!/usr/bin/env python

import os
import sys
import rospkg

import ast

import yaml
import click
from colorama import Fore, Back, Style, init

from Solver import SolverCollection, Logger as log, Chain
from ambf_funcs import loadChainFromAMBF


rospack = rospkg.RosPack()

# =============================================================================== CLI


class PythonLiteralOption(click.Option):
    """Class for using arrays in CLI
    """

    def type_cast_value(self, ctx, value):
        try:
            return ast.literal_eval(value)  # should contain list
        except:
            raise click.BadParameter(value)


@click.command()
@click.pass_context
@click.option('-r', '--robot_file', 'yaml_file', default=os.path.join(rospack.get_path(
    'ambf_controller'), "example_robots", "parallel_structure.yaml"), help='Robot Yaml File path', type=click.Path(exists=True), required=False)
@click.option('-y', '--yaml', 'IS_STANDALONE', default=True, is_flag=True, help="Load Robot from Yaml file instead of AMBF_client.")
@click.option('--fk', cls=PythonLiteralOption, default="[]", help='Joint states(Q1,Q2,..) as array as string i.e. "[1,2,3]"')
@click.option('--ik', cls=PythonLiteralOption, default="[]", help='x,y,z,ax,ay,az of the tip as array as string i.e. "[1.0,2,3,4,5,6]"')
def controller(ctx, yaml_file, IS_STANDALONE, fk, ik):
    # =============================================================================== Initializations
    if IS_STANDALONE:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from : " +
                  Fore.RESET+Style.RESET_ALL+yaml_file)
    else:
        log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from AMBF Simulator." +
                  Fore.RESET+Style.RESET_ALL)

    # =============================================================================== Load all solvers
    solverCollection = SolverCollection()

    solvers = solverCollection.getAllSolvers()
    log.debug("Kinematics Solvers detected :")
    log.debug(solvers)

    # =============================================================================== Loading chain
    # Getting Kinematics chain
    chain = None
    ambf_client = None
    solver_to_use = None

    with open(yaml_file, 'rb') as f:
        data = yaml.load(f)
        if 'solver' in data:
            solver_to_use = data['solver']
        else:
            # Exiting if no solver is defined
            sys.exit("No Solver is defined. Exiting..")

    if IS_STANDALONE:
        chain = Chain(data)
    else:
        ambf_client, chain = loadChainFromAMBF(data)

    # Exiting if no chain is loaded
    if chain == None:
        sys.exit("No Chain detected. Exiting..")

    if(ik == [] and fk == []):  # Nothing to do
        log.debug(Fore.RED+"Either --ik or --fk parameter need to" +
                  " be passed for controller."+Fore.BLUE)
        print(ctx.get_help())
        sys.exit()

    if(ik != [] and fk != []):  # Confused what to do
        log.debug(Fore.RED+"Only one of --ik or --fk parameter need to" +
                  " be passed for controller."+Fore.BLUE)
        print(ctx.get_help())
        sys.exit()

    # Get an instance of the solver to be used
    solver = solverCollection.getSolver(solver_to_use, chain)

    log.debug("Loaded Solver >> " + solver.name)

    if ik != []:  # solving IK
        if len(ik) != 6:  # Validating IK
            log.debug(Fore.RED+"Length of IK parameters should be 6."+Fore.BLUE)
            print(ctx.get_help())
            sys.exit()

        log.debug(solver.solve_for_ik(ik))

    if fk != []:  # solving FK
        log.debug(solver.solve_for_fk(fk))


if __name__ == '__main__':
    controller()
