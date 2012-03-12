import py_indoor_context

from matplotlib.pyplot import *

from training_helpers import *

np.set_printoptions(precision=3)

#w = np.array([1., 3., 4., .1, .5])
w = np.array([1., -1., 0.,])
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)

dataset = [('lab_kitchen1',[60]), ('exeter_mcr1',[10])]

mgr = py_indoor_context.TrainingManager()
load_dataset(mgr, dataset)
#mgr.LoadSequence("lab_kitchen1", [60])
#mgr.LoadSequence("exeter_mcr1", [10])

#mgr.ComputeAllFeatures(True)
mgr.ComputeMockFeatures()

nf = mgr.GetNumFeatures()

inst = mgr.GetInstance(1)
F = inst.GetFeature(1,0)

#means = [ np.mean([inst.GetFeature(i,k) for k in range(2)]) for i in range(nf) ]
#vars = [ np.var([inst.GetFeature(i,k) for k in range(2)]) for i in range(nf) ]

payoffs = py_indoor_context.DPPayoffs()
inst.CompileFeatures(params, payoffs)
soln = mgr.Solve(inst, payoffs)
soln_ys = soln.GetPath()
soln_orients = soln.GetOrients()

gt_ys = inst.GetGroundTruth().GetPath()
loss_terms = inst.GetLossTerms();

soln_ftr = inst.ComputeFeatureForHypothesis(soln);

subplot(311)
imshow(payoffs.Get(0))
plot(range(len(gt_ys)), gt_ys, 'r', linewidth=2)
plot(range(len(soln_ys)), soln_ys, 'w', linewidth=2)

subplot(312)
imshow(imread(inst.GetImagePath()))

subplot(313)
tmp = '/tmp/viz.png'
mgr.OutputSolutionViz(tmp)
imshow(imread(tmp))

show()
