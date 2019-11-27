#!/usr/bin/env python

import sys

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

import yaml
from ambf_client import Client  # Import the Client from ambf_client package
from PyKDL import Rotation
from colorama import Fore, Back, Style, init

from Solver import SolverCollection, Logger as log, Chain, Pose as SolverPose


class AMBF_controller:
    """Main class for ambf controller
    """
    chain = None
    solver = None
    ambf_client = None
    robot_handle = None

    def __init__(self, yaml_file):
        """Initializer for the AMBF class

        Arguments:
            yaml_file {string} -- Path of the YAML file to load for kinematic chain
        """

        data = None

        with open(yaml_file, 'rb') as f:
            data = yaml.load(f)

        if 'solver' in data:
            solver_to_use = data['solver']
        else:
            # Exiting if no solver is defined
            sys.exit("No Solver is defined. Exiting..")

        # =============================================================================== Load data, solver

        # Generating Chain from yaml
        self.chain = Chain(data)

        # Get Solver Collection
        solverCollection = SolverCollection()

        log.debug("Kinematics Solvers detected :")
        log.debug(solverCollection.getAllSolvers())

        # Get an instance of the solver to be used
        self.solver = solverCollection.getSolver(
            solver_to_use, self.chain, strict=True)

        log.debug("Loaded Solver >> " + self.solver.name)

        rospy.Subscriber("/ambf/validate", Pose, self.set_pose_callback)
                
        # =============================================================================== Connect to AMBF

        # Create a instance of the client
        self.ambf_client = Client()

        # Connect the client which in turn creates callable objects from ROS topics
        # and initiates a shared pool of threads for bi-directional communication
        self.ambf_client.connect()

        # Get handle to set joint positions to
        self.robot_handle = self.ambf_client.get_obj_handle(
            self.chain.getBaseBody().name)

        self.robot_tip_handle = self.ambf_client.get_obj_handle(
            self.chain.getTipBody().name)

        # =============================================================================== Start ROS comm

        rospy.Subscriber("/ambf/setPose", Pose, self.set_pose_callback)
        rospy.Subscriber("/ambf/setJointParams",
                         Float64MultiArray, self.set_joint_params_callback)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # print(self.robot_handle.get_all_joint_pos())
        
            # for i in range(4):
            #     print("I:" + str(i) + '>' + str("%5.4f" % self.robot_handle.get_joint_pos(i)))
            #     i = i+1
            # print("------------------------")
            rate.sleep()
            #rospy.sp

    def set_pose_callback(self, msg):
        # Build a Pose
        pose = SolverPose(msg.position.x, msg.position.y, msg.position.z,
                          msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        joint_states = self.solver.solve_for_ik_pos(pose)
        log.info("Joint States Calculated:")
        log.info(joint_states)
        i = 0
        for joint_value in joint_states:
            self.robot_handle.set_joint_pos(i, joint_value)
            i = i+1

    def set_joint_params_callback(self, msg):
        pose = self.solver.solve_for_fk_pos(msg.data)
        log.info("Pose:")
        log.info(pose)
        
        i = 0
        for joint_value in msg.data:
            self.robot_handle.set_joint_pos(i, joint_value)
            i = i+1

if __name__ == '__main__':
    # Globals
    global ambf_client, solver
    # =============================================================================== Initializations
    # rospy.init_node('ambf_controller')

    # Clear the arguments from ROS args
    args = rospy.myargv(argv=sys.argv)

    if len(args) < 2:
        # Loading Default file
        # log.error(Style.BRIGHT+Fore.RED+"No YAML robot file given in the argument" +
        #           Fore.RESET+Style.RESET_ALL)
        sys.exit(Style.BRIGHT+Fore.RED+"No YAML file given in the argument" +
                 Fore.RESET+Style.RESET_ALL)
    else:
        yaml_file = args[1]

    log.debug(Style.BRIGHT+Fore.BLUE+"Loading Robot from : " +
              Fore.RESET+Style.RESET_ALL+yaml_file)

    AMBF_controller(yaml_file)
