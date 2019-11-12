import os
import pkgutil
import inspect

from solvers import *
from Logger import Logger as log
from BaseSolver import BaseSolver
from InvalidChainError import InvalidChainError


class SolverCollection(object):
    """Upon creation, this class will read the solvers package for modules
    that contain a class definition that is inheriting from the Solver class
    """

    def __init__(self, solver_package="Solver.solvers"):
        """Constructor that initiates the reading of all available solvers when an instance of the SolversCollection object is created
        """
        self._solver_package = solver_package
        self.log = log
        self._reload_solvers()

    def getAllSolvers(self):
        """Get list of classes of solvers

        Returns:
            Dict -- Returns an Dictionary with solver name as keys and classes as values 
        """
        self._reload_solvers()
        return {cls.__name__.upper(): cls for cls in BaseSolver.__subclasses__()}
        # return self._classes

    def getSolver(self, solver_name, kinematic_chain, strict=True):
        """Get the solver Oject given the solver name or any other solver, if strict is false.

        Arguments:
            solver_name {string} -- The name of solver to load (Should be valid) or it will throw error.
            kinematic_chain {Chain} -- The chain which will be used by solver to initialize and do chain based validations to decide whether that solver can solve for that chain.

        Keyword Arguments:
            strict {bool} -- If True, it will fail if the specific solver requested does not load. If False, it tries to load any other solver after this in alphabetical order. (default: {True})

        Returns:
            BaseSolver -- Object of the Solver which is loaded (A Subclass of Solver.BaseSolver)
        """
        solvers = self.getAllSolvers()
        solver_name = solver_name.upper().strip()

        if strict:
            if solver_name not in solvers:
                raise TypeError("Solver with given name not found!")
            return solvers[solver_name](kinematic_chain)
        else:
            if solver_name in solvers:
                return solvers[solver_name](kinematic_chain)
            else:
                for cl in solvers:
                    try:
                        return solvers[cl](kinematic_chain)
                    except InvalidChainError:
                        pass
                raise TypeError("No Solver found!")

    def _reload_solvers(self):
        """Reset the list of all solvers and initiate the walk over the main provided solver package to load all available solvers
        """
        self.solvers = []
        self.seen_paths = []
        self._walk_package(self._solver_package)

    def _walk_package(self, package):
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
                    if issubclass(c, BaseSolver) & (c is not BaseSolver):
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
                    self._walk_package(package + '.' + child_pkg)
