from abc import ABCMeta, abstractmethod
import inspect
import os
import pkgutil


class Solver(object):
    """Base class that each solver must inherit from. within this class
    you must define the methods that all of your solvers must implement
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        # capitalize name, add to list
        self.name = name
        #  | 'UNKNOWN'

    @abstractmethod
    def perform_operation(self, argument):
        """The method that we expect all solvers to implement. This is the
        method that our framework will call
        """
        raise NotImplementedError


class SolverCollection(object):
    """Upon creation, this class will read the solvers package for modules
    that contain a class definition that is inheriting from the Solver class
    """

    def __init__(self, solver_package):
        """Constructor that initiates the reading of all available solvers
        when an instance of the SolversCollection object is created
        """
        self.solver_package = solver_package
        self.reload_solvers()

    def reload_solvers(self):
        """Reset the list of all solvers and initiate the walk over the main
        provided solver package to load all available solvers
        """
        self.solvers = []
        self.seen_paths = []
        print('Looking for solvers under package ')
        print(self.solver_package)
        self.walk_package(self.solver_package)

    def apply_all_solvers_on_value(self, argument):
        """Apply all of the solvers on the argument supplied to this function
        """
        print('Applying all solvers on value')
        print(argument)
        for solver in self.solvers:
            print('Applying')
            print(solver.name)
            print('on value')
            print(argument)
            print(' yields value ')
            print(solver.perform_operation(argument))

    def walk_package(self, package):
        """Recursively walk the supplied package to retrieve all solvers
        """
        imported_package = __import__(package, fromlist=['blah'])

        for _, solvername, ispkg in pkgutil.iter_modules(imported_package.__path__, imported_package.__name__ + '.'):
            if not ispkg:
                solver_module = __import__(solvername, fromlist=['blah'])
                clsmembers = inspect.getmembers(solver_module, inspect.isclass)
                for (_, c) in clsmembers:
                    # Only add classes that are a sub class of Solver, but NOT Solver itself
                    if issubclass(c, Solver) & (c is not Solver):
                        print('Found solver class:')
                        print(c.__module__+"."+c.__name__)

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
