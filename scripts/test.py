#!/usr/bin/env python

from Solver import SolverCollection, Logger as log

log.setLevel(log.DEBUG)

s = SolverCollection('Solver.solvers')
try:
    # s.getSolver('delt2a', 's', strict=False)
    solver = s.getSolver('delta ', 's')

    log.debug("LOADED :" + solver.name)
except Exception as err:
    log.debug("ERROR")
    log.debug(err.__class__)
    log.error(err.message)
