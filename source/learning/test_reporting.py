import py_indoor_context

from matplotlib.pyplot import *

import training_helpers

np.set_printoptions(precision=3)

w = np.array([1., -1., 0.,])
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)

w2 = np.array([2., -.5, .1,])
params2 = py_indoor_context.ManhattanHyperParameters(w2, 3., 7.)

train_ids = [20,40,60]
test_ids = [65,70]

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence("lab_kitchen1", train_ids+test_ids)

fm = py_indoor_context.FeatureManager('/tmp')
for i in range(mgr.NumInstances()):
    fm.ComputeMockFeatures(mgr.GetInstance(i))
    fm.CommitFeatures()

train_instances = [mgr.GetInstance(i)
                   for i in range(len(train_ids))]

test_instances = [mgr.GetInstance(i)
                  for i in range(len(train_ids), len(train_ids)+len(test_ids))]

r = training_helpers.Reporter(train_instances, test_instances, fm, 'foo/testexp')
r.add_iteration(params)
r.add_iteration(params2)
r.generate_report(params, extended=True)
