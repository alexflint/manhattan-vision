# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import matplotlib.pyplot as plt

if len(sys.argv) != 3:
    print 'Usage: ',sys.argv[0],' DEPTH.pickled LABELLING.pickled'
    exit(-1)

losses = ['depth_error',
#          'labelling_error',
          'twolabelling_error']
errors = {loss:{} for loss in losses}

depthdata = cPickle.load(open(sys.argv[1]))
lbldata = cPickle.load(open(sys.argv[2]))

depth_xs = []
depth_ys = []
lbl_xs = []
lbl_ys = []

for key,record in depthdata['results'].iteritems():
    depth_ys.append(record['twolabelling_error'] * 100.)
    depth_xs.append(record['depth_error'] * 100.)

for key,record in lbldata['results'].iteritems():
    lbl_xs.append(record['twolabelling_error'] * 100.)
    lbl_ys.append(record['depth_error'] * 100.)

plt.rc('font', size=14.)

plt.clf()
d=plt.scatter(depth_xs, depth_ys, c='r', marker='o', s=100, linewidth=0)
l=plt.scatter(lbl_xs, lbl_ys, c='b', marker='x', s=150, linewidth=3)
plt.ylabel('Depth error (%)', fontsize=18.)
plt.xlabel('Labelling error (%)', fontsize=18.)
plt.xlim(0, 50)
plt.ylim(0, 50)
plt.legend((d,l),('Trained w.r.t. depth loss','Trained w.r.t. labelling loss'))
plt.savefig('comparative_scatter.pdf')
#plt.show()
