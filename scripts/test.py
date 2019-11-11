#!/usr/bin/env python

from Solver import SolverCollection

s = SolverCollection('Solver.solvers')
try:
    # s.getSolver('delt2a', 's', strict=False)
    solver = s.getSolver('delta ', 's')

    print("LOADED :" + solver.name)
except Exception as err:
    print("ERROR")
    print(err.__class__)
    print(err.message)
