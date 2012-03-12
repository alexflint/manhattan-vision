import numpy as np
from matplotlib.pyplot import *

import py_indoor_context
import training_helpers

np.set_printoptions(precision=3)

#w = np.array([0., 0., 1.,])
w = np.array([0.331, -0.662,  0.004])
#w = np.array([229.582,  147.414,  -11.827,   75.342,   87.373,   27.623,   37.698])
#params = py_indoor_context.ManhattanHyperParameters(w, 400., 600.)
params = py_indoor_context.ManhattanHyperParameters(w, 13.462474823, 0.)

mgr = py_indoor_context.TrainingManager()
#mgr.LoadSequence('lab_kitchen1', [25]) #range(10,80,10))
mgr.LoadSequence('exeter_mcr1', [35]) #range(10,80,10))

#mgr.ComputeAllFeatures([-5,-1,1,5], True)
mgr.ComputeMockFeatures()

instances = [ mgr.GetInstance(i) for i in range(mgr.GetNumInstances()) ]

sum_err = 0.
for inst in instances:
    inst.ConfigureLabellingLoss()

    print '\nFrame ',inst.GetFrameId()
    payoffs = py_indoor_context.DPPayoffs()
    inst.CompileFeatures(params, payoffs)
    hyp = mgr.Solve(inst, payoffs)

    # Check score
    w = training_helpers.get_psi_from_params(params)
    ftr = training_helpers.get_feature(inst, hyp)
    print 'w: ',w
    print 'ftr: ',ftr
    score = np.dot(ftr, w)
    other_score = payoffs.ComputeScore(hyp)
    err = np.abs(score - other_score) / other_score
    sum_err += err
    print '  ftr <dot> w: ',score
    print '  ComputeScore(...): ',other_score
    print '  Error: %f%%' % (err*100.)

n = mgr.GetNumInstances()
print 'Average error: %f%%' % (sum_err * 100. / n)

