"""Linear path model."""

import numpy as np

def L1(x,xx):
    return abs(x-xx)

def L2(x,xx):
    return (x-xx)*(x-xx)

# This needs to be updated to include transitions:
#def compute_path_loglik(F, path, w):
#    return np.sum(np.dot(F[[path,range(F.shape[1])]], w))

def compute_loss(gt, estimated, fi):
    if np.ndim(gt) != 1:
        raise Exception('"gt" should be a 1-dimensional')
    if np.ndim(estimated) != 1:
        raise Exception('"estimated" should be a 1-dimensional')
    return np.sum([fi(xi, xihat) for (xi,xihat) in zip(gt,estimated)])

def compute_data_terms(F, w):
    return np.dot(F, w)

def compute_loss_terms(gt, shape, lossfunc):
    assert(np.shape(shape) == (2,))
    return np.array([[lossfunc(i, gt[j]) 
                      for j in range(shape[1])]
                     for i in range(shape[0])])

def compute_loss_augmented_terms(F, data_weights, gt, fi):
    assert np.ndim(F) == 3
    assert np.shape(gt) == (np.size(F,1),)
    assert np.shape(data_weights) == (np.size(F,2),)
    data_terms = compute_data_terms(F, data_weights)
    loss_terms = compute_loss_terms(gt, F.shape[:2], fi)
    return data_terms + loss_terms

def compute_data_features(F, states):
    return np.sum(F[[states,range(F.shape[1])]], axis=0)

def compute_transition_features(orients):
    if np.ndim(orients) != 1:
        raise Exception('orients should be 1-dimensional')
    # This feature function assumes a transition matrix that looks like:
    # [ g0 g1 g2 ]
    # [ g1 g0 g1 ]
    # [ g2 g1 g0 ]
    # where the weight vector looks like [ g0 g1 g2 ]
    jumps = [np.abs(orients[i] - orients[i+1]) for i in range(len(orients)-1)]
    counts = np.bincount(jumps)
    counts = np.append(counts, [0]*(3-len(counts)))   # ensure its length is >= 3
    if len(counts) !=3:
        raise Exception('There were more than 3 bins, so jumps of > 2')
    return counts

def compute_path_features(F, soln):
    if len(soln) != 2:
        raise Exception('path should be a pair (states,orients)')
    states,orients = soln
    return np.append(compute_data_features(F, states),
                     compute_transition_features(orients))
