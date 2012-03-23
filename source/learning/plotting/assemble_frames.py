# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import matplotlib.pyplot as plt
import shutil
import stuff

for key,experiment in stuff.Experiments.iteritems():
    datafile = os.path.join(experiment, 'holdout_performance.pickled')
    d = cPickle.load(open(datafile, 'r'))

    for record in d['results'].itervalues():
        seq = record['sequence']
        idx = record['frame_id']
        srcpath = os.path.join(experiment, 'out/%s_frame%03d_soln.png' % (seq,idx))
        destpath = 'out/%s_frame%03d_%s.png' % (seq,idx,key)
        shutil.copy(srcpath, destpath)
