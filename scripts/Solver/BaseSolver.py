from abc import ABCMeta, abstractmethod

from logger import logger as log


class BaseSolver(object):
    """Base class that each solver must inherit from.
    """
    __metaclass__ = ABCMeta

    def __init__(self, chain):
        """This method will initialize super class instance of solver

        Arguments:
            chain {Chain} -- Chain on which the IK & FK is to be solved
        """
        self.chain = chain
        # do any more initializations here
        self.init(chain)

    @abstractmethod
    def init(self, chain):
        """This method will be called after the initialization of the
        base class is completed. This function will give the chain as 
        a parameter.

        ** NOTE: `self.chain` can be used to access this chain **

        Arguments:
            chain {Chain} -- Chain on which the IK & FK is to be solved

        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError

    @abstractmethod
    def solve_for_fk(self, joint_states):
        """This method is expected to be implemented by all solvers. This is the
        method is called whenever the Forward kinematics is required to be solved.

        Arguments:
            joint_states {array} -- Array of all the joint states

        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError

    @abstractmethod
    def solve_for_ik(self, tip_6DOF):
        """This method is expected to be implemented by all solvers. This is the
        method is called whenever the Inverse kinematics is required to be solved.

        Arguments:
            tip_6DOF {array/dict} -- the Tip 6DOF (x,y,z,ax,ay,az)

        Raises:
            NotImplementedError: This function needs to be implemented
            by the sub class.
        """
        raise NotImplementedError
