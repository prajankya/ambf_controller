from abc import ABCMeta, abstractmethod
import inspect
import os
import sys
import pkgutil

from logger import logger as log


class Solver(object):
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


class _SolverCollection(object):
    """Upon creation, this class will read the solvers package for modules
    that contain a class definition that is inheriting from the Solver class
    """

    def __init__(self, solver_package):
        """Constructor that initiates the reading of all available solvers
        when an instance of the SolversCollection object is created
        """
        self._solver_package = solver_package
        self.log = log
        self._reload_solvers()

    def getAllSolvers(self):
        # all_my_base_classes = {
        #     cls.__name__.upper(): cls for cls in Solver.__subclasses__()}
        return self._classes

    def _reload_solvers(self):
        """Reset the list of all solvers and initiate the walk over the main
        provided solver package to load all available solvers
        """
        self.solvers = []
        self.seen_paths = []
        # self.debug('Looking for solvers under package: ' +
        #            self._solver_package)
        self.walk_package(self._solver_package)

    def walk_package(self, package):
        """Recursively walk the supplied package to retrieve all solvers
        """
        imported_package = __import__(package, fromlist=['blah'])

        self._classes = {}

        for _, solvername, ispkg in pkgutil.iter_modules(imported_package.__path__, imported_package.__name__ + '.'):
            if not ispkg:
                solver_module = __import__(solvername, fromlist=['blah'])
                clsmembers = inspect.getmembers(solver_module, inspect.isclass)
                for (_, c) in clsmembers:
                    # Only add classes that are a sub class of Solver, but NOT Solver itself
                    if issubclass(c, Solver) & (c is not Solver):
                        # self.debug('Found solver class: ' +
                        #            c.__module__+"."+c.__name__)
                        self._classes[c.__name__.upper()] = c

        # Now that we have looked at all the modules in the current package, start looking
        # recursively for additional modules in sub packages
        all_current_paths = []
        if isinstance(imported_package.__path__, str):
            all_current_paths.append(imported_package.__path__)
        else:
            all_current_paths.extend([x for x in imported_package.__path__])

        for pkg_path in all_current_paths:
            if pkg_path not in self.seen_paths:
                self.seen_paths.append(pkg_path)

                # Get all sub directory of the current package path directory
                child_pkgs = [p for p in os.listdir(
                    pkg_path) if os.path.isdir(os.path.join(pkg_path, p))]

                # For each sub directory, apply the walk_package method recursively
                for child_pkg in child_pkgs:
                    self.walk_package(package + '.' + child_pkg)
