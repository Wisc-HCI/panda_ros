"""Peforms random restart in optimization
to avoid local optima

 Last Updated: 03/18/2019
"""

__author__ = "Mike Hagenow"

from scipy.optimize import minimize as minimizeScipy
import numpy as np


def minimize(objective_func,num_params,method='BFGS',jac=None,options = None,numRestarts = 5):
    """
        Calls scipy minimize with random restarts and returns the best minimization

        :param objective_func:
        :param init_cond:
        :param method:
        :param jac:
        :param options:
        :param numRestarts:
        :type objective_func:
        :type init_cond:
        :type method:
        :type jac:
        :type options:
        :type numRestarts:
        :returns: bestOpt
        :rtype: OptimizeResult (scipy)
    """

    #Start at max float so firsr restart becomes minimum
    currMin = float("inf")


    for ii in range(0,numRestarts):
        # Get a new random set of conditions for each restart
        init_cond = np.random.rand(num_params)

        tempOpt = minimizeScipy(objective_func,init_cond,method='BFGS',jac=jac,options=options)

        #Save best minimization (based on function value)
        if tempOpt.fun < currMin:
            currMin = tempOpt.fun
            bestOpt=tempOpt

    return bestOpt


