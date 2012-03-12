# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import stuff

def ComputePerformance(datafile):
    losses = ['depth_error',
              'twolabelling_error']
    errors = {loss:{} for loss in losses}

    with open(datafile) as f:
        d = cPickle.load(f)
        for key,record in d['results'].iteritems():
            seq = record['sequence']
            for loss in losses:
                if seq not in errors[loss]:
                    errors[loss][seq] = []
                errors[loss][seq].append(record[loss])
    
    for loss in losses:
        print '\n***',loss
        allerrs = []
        for seq,errs in errors[loss].iteritems():
            av = np.mean(errs)
            std = np.std(errs)
            allerrs.append(av)
            print '{:<30}{:10.1f} {:10.1f}'.format(seq, av*100, std*100)

        av = np.mean(allerrs)
        std = np.std(allerrs)
        print '{:<30}{:10.1f} {:10.1f}'.format('AVERAGE', av*100, std*100)



if len(sys.argv) > 2:
    print 'Usage: ',sys.argv[0],' RESULTS.pickled'
    exit(-1)

elif len(sys.argv) == 2:
    ComputePerformance(sys.argv[1])

else:
    order = [ 'mview_depth',
              'mview_lbl',
              'iccv',
              'sview_depth',
              'sview_lbl',
              'eccv'
              ]

    for name in order:
        path = os.join(stuff.Experiments[name], 'holdout_performance.pickled')
        ComputePerformance(path)
