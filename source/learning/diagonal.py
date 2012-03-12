"""Viterbi algorithm."""

import numpy as np
import numpy.random as rnd

from data_generation import *

nexamples = 10
noutliers = 0

# Sample some training examples
def generate_training_data(nexamples, noutliers=0, ntimesteps=5, nstates=4,
                           T=None,
                           seed=1234):
    if T is None:
        T = 3 + np.eye(3)

    data = []
    rnd.seed(seed)  # for repeatability

    # Generate ordinary examples
    for i in range(nexamples):
        gt = sample_path(nstates, ntimesteps, T)
        ftrs = sample_features(gt[0], ntimesteps, nstates)
        data.append((ftrs, gt))

    # Generate outliers
    for i in range(noutliers):
        gt = sample_path(nstates, ntimesteps, T)
        ftrs = sample_outlier_features(gt[0], ntimesteps, nstates)
        data.append((ftrs, gt))

    return data

def generate_trivial_data(nexamples, ntimesteps=5, nstates=4,
                          T=None,
                          seed=1234):
    if T is None:
        T = np.array([[0,1,0],[0,1,0],[0,1,0]])

    data = []
    rnd.seed(seed)  # for repeatability

    for i in range(nexamples):
        gt = sample_path(nstates, ntimesteps, T)
        ftrs = np.zeros((nstates, ntimesteps, 1))  # all are zero
        ftrs[gt[0][0], 0, 0] = 1.  # only label the beginning state
        data.append((ftrs, gt))

    return data


def generate_confounding_data(nexamples, ntimesteps=5, nstates=4,
                              T=None,
                              seed=1234):
    if T is None:
        T = np.array([[0,1,0],[0,1,0],[0,1,0]])
    T_confuse = 1-np.eye(3)

    data = []
    rnd.seed(seed)  # for repeatability

    for i in range(nexamples):
        gt = sample_path(nstates, ntimesteps, T)
        confuser = sample_path(nstates, ntimesteps, T_confuse)
        ftrs = np.zeros((nstates, ntimesteps, 1))  # all are zero
        
        ftrs[[ gt[0], range(ntimesteps) ]] = 1
        ftrs[[ confuser[0], range(ntimesteps) ]] = 1

        data.append((ftrs, gt))

    return data



# Upack a weight vector into a weight for data features and a transition matrix
def unpack_weights(w):
    if np.ndim(w) != 1:
        raise Exception('w should be 1-dimensional')
    data_weights = w[:-3]
    T = np.array([[ w[-3], w[-2], w[-1] ],
                  [ w[-2], w[-3], w[-2] ],
                  [ w[-1], w[-2], w[-3] ]])
    return (data_weights,T)

def solve(A, T):
    assert(np.ndim(A) == 2)
    assert(np.shape(T) == (3,3))

    nstates,ntimesteps = A.shape
    norients = 3

    best = -np.inf * np.ones((nstates, ntimesteps, norients))
    src = np.empty((nstates, ntimesteps, norients, 2), int)  # last dim to store tuples
    for a in range(3):
        best[:,0,a] = A[:,0]
    for x in range(1, ntimesteps):
        for y in range(nstates):
            for a in range(3):
                yy = y-(a-1)
                if yy >= 0 and yy < nstates:
                    for aa in range(3):
                        # yes we should use the same yy here as above
                        hyp = best[yy, x-1, aa] + A[y,x] + T[aa,a]
                        if hyp > best[y,x,a]:
                            best[y,x,a] = hyp
                            src[y,x,a] = (yy,aa)

    states = [0] * ntimesteps
    orients = [0] * ntimesteps

    imax = np.argmax(best[:,ntimesteps-1])
    states[ntimesteps-1] = imax / norients
    orients[ntimesteps-1] = imax % norients

    for i in range(ntimesteps-2,-1,-1):
        states[i],orients[i] = src[ states[i+1], i+1, orients[i+1] ]
    
    orients = list(np.subtract(orients, 1))  # correct for indexing stuff

    return (states,orients)
