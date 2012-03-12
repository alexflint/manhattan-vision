# Helpers for generating training data

import numpy as np
import numpy.random as rnd

# Sample from a (possibly not normalized) PDF over discrete values
def sample_from_pdf(pdf):
    cdf = np.cumsum(pdf)
    return np.searchsorted(cdf, rnd.random()*cdf[-1])

# Sample a path from the prior
def sample_path(nstates, ntimesteps, T):
    path = np.zeros(ntimesteps, int)
    dirs = np.zeros(ntimesteps, int)

    # Pick a starting row and direction
    path[0] = rnd.randint(nstates)
    dirs[0] = 0
    for c in range(1,ntimesteps):
        valid_dirs = [ path[c-1]>0, True, path[c-1]<nstates-1 ]
        pdf = T[dirs[c-1]+1] * np.asarray(valid_dirs, int)
        dirs[c] = sample_from_pdf(pdf)-1
        path[c] = path[c-1] + dirs[c]

    return path,dirs

# Sample features given a ground truth path. The feature vector looks like:
# [   1 if this cell on true path else 0  ] 
# [   0 if this cell on true path else 1  ]
# [   1.5                                 ]
# [   random value in (0, .1)             ]
def sample_features(path, ntimesteps, nstates):
    return np.array([[ [ int(r == path[c]),
                         int(r != path[c]),
                         1.5,
                         rnd.rand()/10,
                         ]
                      for c in range(ntimesteps)]
                     for r in range(nstates)])

# Sample features from a different distrubtion given a ground truth
# path. We use these kind of features as "outliers" but there is
# nothing special about them, they're just different to the ones
# above.
def sample_outlier_features(path, ntimesteps, nstates):
    return np.array([[ [0, 0, 0, int(r == path[c])]
                      for c in range(ntimesteps)]
                     for r in range(nstates)])
