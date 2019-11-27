from abc import ABCMeta, abstractmethod

from Logger import Logger as log


class BaseSolver(object):
    """Base class that each solver must inherit from.
    """
    __metaclass__ = ABCMeta
    _joint_states = []

    def __init__(self, chain):
        """This method will initialize super class instance of solver

        Arguments:
            chain {Chain} -- Chain on which the IK & FK is to be solved
        """
        self.name = self.__class__.__name__.upper()
        self.chain = chain
        self.init(chain)

    @abstractmethod
    def init(self, chain):
        """This method will be called after the initialization of the base class is completed. This function will give the chain as a parameter.

        ** NOTE: `self.chain` can be used to access this chain **
        ** NOTE: `self.name` can be used to access the name **

        Arguments:
            chain {Chain} -- Chain on which the IK & FK is to be solved

        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError

    @abstractmethod
    def solve_for_fk_pos(self, joint_states):
        """This method is expected to be implemented by all solvers. This is the
        method is called whenever the Forward kinematics is required to be solved.

        Arguments:
            joint_states {array} -- Array of all the joint states
        Returns:
            pose {Pose} -- Returns Tip Pose (of type Solver.Pose)
        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError

    @abstractmethod
    def solve_for_ik_pos(self, tip_Pose):
        """This method is expected to be implemented by all solvers. This is the
        method is called whenever the Inverse kinematics is required to be solved.

        Arguments:
            tip_Pose {Pose} -- the Tip Pose (x,y,z,ax,ay,az) of type Solver.Pose
        Returns:
            joint_states {array} -- Array of all the joint states
        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError

    def set_current_jointstates(self, joint_states):
        """Set Joint states, so any solver can use the current joint states from 
        ambf for its calculation
        
        Arguments:
            joint_states {Array} -- Array of jointstates
        """
        self._joint_states = joint_states

    def get_current_jointstates(self):
        """Get the current joint states, it will return empty array if no
        Joint states were ever set by calling "set_current_jointstates".
        
        Returns:
            Array -- Array of jointstates
        """
        return self._joint_states
    # Velocity solvers are not in Current scope of the project
    # def solve_for_ik_vel(self)
