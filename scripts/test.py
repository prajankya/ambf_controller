#!/usr/bin/env python

from Solver import SolverCollection
from checker import Checker
s = SolverCollection('Solver.solvers')
q = s.getAllSolvers()
print(q)
