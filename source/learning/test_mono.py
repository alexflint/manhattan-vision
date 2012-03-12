import py_indoor_context

from matplotlib.pyplot import *

import training_helpers

np.set_printoptions(precision=3)

w = np.zeros(9)
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)

w2 = np.arange(9, dtype=float)
params2 = py_indoor_context.ManhattanHyperParameters(w2, 3., 7.)

train_ids = [20,40,60]
test_ids = [65,70]

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence("lab_kitchen1", train_ids+test_ids)
mgr.ComputeMonoFeatures('rgb', True)

train_instances = [mgr.GetInstance(i)
                   for i in range(len(train_ids))]

test_instances = [mgr.GetInstance(i)
                  for i in range(len(train_ids), len(train_ids)+len(test_ids))]

r = training_helpers.Reporter(train_instances, test_instances, 'foo/testexp')
r.add_iteration(params)
r.add_iteration(params2)
r.generate_report(params, extended=True)
