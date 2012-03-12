# Compute the average per-sequence performance

import os
import sys
import numpy as np
import cPickle
import itertools
import matplotlib.pyplot as plt
import stuff

for name,dir in stuff.Experiments:

    path = os.path.join(dir, 'holdout_performance.pickled')
    
    print path

    with open(path) as f:
        d = cPickle.load(f)
        ftrlen = len(d['features'])
        if ftrlen > 9:
            ftr_is = [0,3,6,9,12,15,72,75,78]
        else:
            ftr_is = range(ftrlen)
        print ftr_is
        ws = [ [ r['weights'][i] for r in d['params_history'] ]
               for i in ftr_is ]
        ws.append([ r['corner_penalty'] for r in d['params_history'] ])
        ws.append([ r['occlusion_penalty'] for r in d['params_history'] ])
        
        ftrnames = [ d['features'][i] for i in ftr_is ]
        ftrnames.append('Corner Penalty')
        ftrnames.append('Occlusion Penalty')

        plt.clf()
        plt.rc('font', size=12.)
        plt.figure(figsize=(12,4), dpi=100)
        plt.subplot2grid((1,10), (0,0), colspan=7) # make space for legend

        for w in ws:
            w /= np.mean(w)
            plt.plot(w)

        l=plt.legend(ftrnames,
                     bbox_to_anchor=(1.05,1),
                     loc=2,
                     borderaxespad=0.)
        l.draw_frame(False)

        plt.xlim(xmin=0)
        plt.ylim(-4,4)
        plt.xlabel('Iteration', fontsize=14.)
        plt.ylabel('Component Weight', fontsize=14.)

        outpath = 'out/psi_evolution_%s.pdf' % name
        plt.savefig(outpath)
        print outpath
