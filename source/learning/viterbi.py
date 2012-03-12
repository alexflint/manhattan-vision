"""Viterbi algorithm."""

import numpy as np
import numpy.random as rnd

from data_generation import *

def generate_training_data(nexamples, noutliers=0, ntimesteps=5, nstates=4):
    print 'Generating training data';
    rnd.seed(1234)  # for repeatability

    # Generate examples
    paths = [ rnd.randint(0, nstates, ntimesteps) for i in range(nexamples) ]
    ftrs = [ sample_features(path, ntimesteps, nstates) for path in paths ]

    # Generate outliers
    outlier_paths = [ rnd.randint(0, nstates, ntimesteps) for i in range(noutliers) ]
    outlier_ftrs = [ sample_outlier_features(path, ntimesteps, nstates)
                     for path in outlier_paths ]

    paths += outlier_paths
    ftrs += outlier_ftrs
    
    # Zip them together
    return zip(ftrs, paths)

def solve(A):
    nstates,ntimesteps = A.shape
    T = np.ones((nstates, nstates))

    assert(np.ndim(A) == 2)
    assert(np.ndim(T) == 2)

    best = -np.inf * np.ones_like(A)
    src = -1 * np.ones_like(A).astype(int)
    best[:,0] = A[:,0]
    for i in range(1, ntimesteps):
        for x in range(nstates):
            for xx in range(nstates):
                hyp = best[xx,i-1] + A[x,i] + T[xx,x]
                if hyp > best[x,i]:
                    best[x,i] = hyp
                    src[x,i] = xx

    bt = [0] * ntimesteps
    bt[ntimesteps-1] = np.argmax(best[:,ntimesteps-1])
    for i in range(ntimesteps-2,-1,-1):
        bt[i] = src[ bt[i+1], i+1 ]

    return bt
