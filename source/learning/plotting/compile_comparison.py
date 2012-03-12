# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import matplotlib.pyplot as plt
import csv

dirs = ['experiments/feb21_depthloss_allfeatures_4offsets',
        'experiments/feb21_labellingloss_allfeatures_4offsets',
        'experiments/feb29_iccvparams'
        ]

if len(sys.argv) != 1:
    print 'Usage:',sys.argv[0]
    exit(-1)

datafile = 'holdout_performance.pickled'
pattern = 'out/%s_frame%03d_soln.png'

losses = ['depth_error',
          'twolabelling_error']

records = {}

for experiment in dirs:
    path = os.path.join(experiment,datafile)
    print path
    with open(path) as f:
        data = cPickle.load(f)
        for k,r in data['results'].iteritems():
            if k not in records:
                records[k] = []
            records[k].append(r)

w = csv.writer(open('compilation.csv','w'))

w.writerow(['Name',
            'Fdepth (depth error)',
            'Fdepth (label error)',
            'Flabel (depth error)',
            'Flabel (label error)',
            'iccv (depth error)',
            'iccv (label error)'
            ])

for k,r in records.iteritems():
    w.writerow([k,
                r[0]['depth_error'],
                r[0]['twolabelling_error'],
                r[1]['depth_error'],
                r[1]['twolabelling_error'],
                r[2]['depth_error'],
                r[2]['twolabelling_error']
               ])
