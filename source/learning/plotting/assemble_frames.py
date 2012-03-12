# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import matplotlib.pyplot as plt
import shutil
import stuff

if len(sys.argv) < 3 or len(sys.argv) > 4:
    print 'Usage:',sys.argv[0],' SEQUENCE FRAMEID [DEST]'
    exit(-1)

seq = sys.argv[1]
idx = int(sys.argv[2])
dest = None if len(sys.argv) < 4 else sys.argv[3]


titles = ['Trained w.r.t. depth loss',
          'Trained w.r.t. labelling loss',
          'ICCV'
          ]

keys = ['depthtrained',
        'lbltrained',
        'iccv'
        ]

pattern = 'out/%s_frame%03d_soln.png'

for i,experiment in enumerate(dirs):
    path = os.path.join(experiment, pattern % (seq,idx))
    if dest is None:
        im = plt.imread(path)
        plt.subplot(1, 3, i+1)
        plt.imshow(im)
        plt.title(titles[i])
    else:
        destpath = os.path.join(dest, '%s_frame%03d_%s.png' % (seq,idx,keys[i]))
        shutil.copy(path, destpath)

if dest is None:
    plt.show()
