import numpy as np
from matplotlib.pyplot import *

import py_indoor_context
from training_helpers import *

# Compute label error discounting differences in vertical orientation
def compute_2label_loss(A,B):
    return np.sum(np.clip(A,1,2) != np.clip(B,1,2))

# Compute label error discounting differences in vertical orientation
def compute_depth_loss(A,B):
    return np.sum(np.abs(A-B) / np.maximum(A,B))

np.set_printoptions(precision=3)

w = np.array([0., 0., 1.,])
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence('lab_kitchen1', [10])
instances = [ mgr.GetInstance(i) for i in range(mgr.GetNumInstances()) ]

mgr.ComputeMockFeatures()
nf = mgr.GetNumFeatures()

sum_lbl_err = 0.
sum_depth_err = 0.

inst = instances[0]

print '\nFrame ',inst.GetFrameId()
payoffs = py_indoor_context.DPPayoffs()
inst.CompileFeatures(params, payoffs)
hyp = mgr.Solve(inst, payoffs)
hyp_labels = mgr.GetSolutionLabels()
hyp_depths = mgr.GetSolutionDepths()
hyp_path = hyp.GetPath()
#hyp_grid_labels = inst.ComputeGridLabels(hyp);
#print 'HYP DEPTHS...'
#hyp_grid_depths = inst.ComputeGridDepths(hyp);

gt = inst.GetGroundTruth()
gt_labels = inst.GetGroundTruthLabels()
gt_depths = inst.GetGroundTruthDepths()
gt_path = inst.GetGroundTruth().GetPath()
#gt_grid_labels = inst.ComputeGridLabels(gt);
#print 'GT DEPTHS...'
#gt_grid_depths = inst.ComputeGridDepths(gt);

print 'LOSS TERMS...'
inst.ConfigureDepthLoss()
depth_loss = inst.ComputeLoss(hyp)
depth_loss_terms = inst.GetLossTerms()
true_depth_loss = compute_depth_loss(gt_depths, hyp_depths)                  
#true_depth_loss = compute_depth_loss(gt_grid_depths, hyp_grid_depths)                  
depth_err = np.abs(depth_loss - true_depth_loss) / true_depth_loss
sum_depth_err += depth_err
print 'Depth loss for h';
print '  ComputeLoss: ',depth_loss
print '  True loss: ',true_depth_loss
print '  Error: %.1f%%' % (depth_err*100.)

print 'PYTHON ERROR...'
x=0
for y in range(0,depth_loss_terms.shape[0],100):
    gt_d = gt_depths[y,x]
    hyp_d = hyp_depths[y,x]
    contrib = np.abs(gt_d - hyp_d) / np.maximum(gt_d, hyp_d)
    print ('Depth at (%d,%d)  HYP=%.3f  GT=%.3f  (contrib=%.3f)' %
           (x, y, hyp_depths[y,x], gt_depths[y,x], contrib))

#clf()

#subplot(321)
#imshow(gt_grid_labels)
#subplot(322)
#imshow(hyp_grid_labels)

#subplot(323)
#imshow(gt_grid_depths)
#subplot(324)
#imshow(hyp_grid_depths)

#subplot(325)
#imshow(depth_loss_terms)
#show()
