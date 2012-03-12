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

#assert(False), 'Removing conditioning factors in TrainingInstance::Configure*Loss first'

np.set_printoptions(precision=3)

w = np.array([0., 0., 1.,])
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence('lab_kitchen1', range(10,80,10))
instances = [ mgr.GetInstance(i) for i in range(mgr.GetNumInstances()) ]

mgr.ComputeMockFeatures()
nf = mgr.GetNumFeatures()

sum_lbl_err = 0.
sum_depth_err = 0.

for inst in instances:
    print '\nFrame ',inst.GetFrameId()
    payoffs = py_indoor_context.DPPayoffs()
    inst.CompileFeatures(params, payoffs)
    hyp = mgr.Solve(inst, payoffs)
    hyp_labels = mgr.GetSolutionLabels()
    hyp_depths = mgr.GetSolutionDepths()
    #hyp_grid_depths = inst.ComputeGridDepths(hyp);
    hyp_path = hyp.GetPath()

    gt = inst.GetGroundTruth()
    gt_labels = inst.GetGroundTruthLabels()
    gt_depths = inst.GetGroundTruthDepths()
    #gt_grid_depths = inst.ComputeGridDepths(gt);
    gt_path = inst.GetGroundTruth().GetPath()

    # Check labelling loss terms
    inst.ConfigureLabellingLoss(1.)
    lbl_loss = inst.ComputeLoss(hyp)
    lbl_loss_terms = inst.GetLossTerms()
    true_lbl_loss = compute_2label_loss(gt_labels, hyp_labels)
    lbl_err = np.abs(lbl_loss - true_lbl_loss) / true_lbl_loss
    gt_lbl_loss = inst.ComputeLoss(gt)
    sum_lbl_err += lbl_err
    print 'Labelling loss for h';
    print '  ComputeLoss: ',lbl_loss
    print '  True loss: ',true_lbl_loss
    print '  Error: %.1f%%' % (lbl_err*100.)
    print '  Loss of GT: %f' % gt_lbl_loss

    inst.ConfigureDepthLoss(1.)
    depth_loss = inst.ComputeLoss(hyp)
    depth_loss_terms = inst.GetLossTerms()
    true_depth_loss = compute_depth_loss(gt_depths, hyp_depths)
    depth_err = np.abs(depth_loss - true_depth_loss) / true_depth_loss
    gt_depth_loss = inst.ComputeLoss(gt)
    sum_depth_err += depth_err
    print 'Depth loss for h';
    print '  ComputeLoss: ',depth_loss
    print '  True loss: ',true_depth_loss
    print '  Error: %.1f%%' % (depth_err*100.)
    print '  Loss of GT: %f' % gt_depth_loss

    #tmp = '/tmp/viz.png'
    #mgr.OutputSolutionViz(tmp)

n = mgr.GetNumInstances()
print 'Average error in label losses: %.1f%%' % (sum_lbl_err * 100. / n)
print 'Average error in depth losses: %.1f%%' % (sum_depth_err * 100. / n)
