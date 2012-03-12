""" Naive QP solver """

import itertools

import numpy as np
import numpy.linalg

# An exponential time QP solver
def naive_qp_solve(obj, cs, bs):
    d = len(cs[0])
    best_f = np.inf
    best_x = None
    for comb in itertools.combinations(zip(cs, bs), d):
        cset, bset = zip(*comb)
        try:
            # Find intersection
            # Something a bit nicer here please...
            # Negative below is because b means (c.x + b > 0  =>  c.x = -b)
            x = numpy.linalg.solve(np.array(cset), -np.array(bset))
            f = obj(x)
            if f < best_f:
                # Do any other constraints invalidate this x?
                feasible = True
                for c,b in zip(cs,bs):
                    if np.dot(c,x)+b < -1e-6:
                        feasible = False
                        break

                # We have a winner
                if (feasible):
                    best_f = f
                    best_x = x
        except numpy.linalg.LinAlgError:
            pass   # Just ignore and move on

    if best_x is None:
        print 'Failed to find a feasible point'

    return best_x
