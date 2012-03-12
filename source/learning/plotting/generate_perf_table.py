# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import stuff

def ComputePerformance(datafile, loss):
    errors = {}

    with open(datafile) as f:
        d = cPickle.load(f)
        for key,record in d['results'].iteritems():
            seq = record['sequence']
            if seq not in errors:
                errors[seq] = []
            errors[seq].append(record[loss])

        return { seq: np.mean(errs) for seq,errs in errors.iteritems() }


alg_order = [ 'mview_depth',
              'mview_lbl',
              'iccv',
              'sview_depth',
              'sview_lbl',
              'eccv'
              ]

seq_order = [
    ('ground', 'lab_ground1'),
    ('foyer1', 'lab_foyer1'),
    ('foyer2', 'lab_foyer2'),
    ('corridor', 'som_corr1'),
    ('mcr', 'exeter_mcr1'),
    ('kitchen', 'lab_kitchen1'),
]

losses = ['depth_error',
          'twolabelling_error']

for loss in losses:
    print '\n',loss,'\n---'

    if loss == 'twolabelling_error':
        temp = alg_order[3]
        alg_order[3] = alg_order[4]
        alg_order[4] = temp

    err_table = {}
    for alg in alg_order:
        path = os.path.join(stuff.Experiments[alg], 'holdout_performance.pickled')
        err_table[alg] = ComputePerformance(path, loss)

    for seqname,seq in seq_order:
        row =  ['\\tt{'+seqname+'}'] + [ err_table[alg][seq]*100. for alg in alg_order ]
        print '{:<14}  &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} \\\\'.format(*row)

    print '\midrule \\\\'
    avs = ['Average'] + [ np.mean(err_table[alg].values())*100. for alg in alg_order ]
    print '{:<14}  &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} &{:6.1f} \\\\'.format(*avs)
