import numpy as np
import matplotlib.pyplot as plt

import py_indoor_context
import training_helpers

np.set_printoptions(precision=3)

w = np.array([1., -1., 0.,])
params = py_indoor_context.ManhattanHyperParameters(w, 4., 6.)


mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence("lab_kitchen1", [70])
mgr.ComputeMockFeatures()
inst = mgr.GetInstance(0)

payoffs = py_indoor_context.DPPayoffs()
inst.CompileFeatures(params, payoffs)

gt_path = inst.GetGroundTruth().GetPath()

solver = py_indoor_context.ManhattanInference()
soln = solver.Solve(inst, payoffs)
soln_path = soln.GetPath()

# Draw
vs = payoffs.Get(0) + payoffs.Get(1)
plt.clf()
plt.title('Solution')
plt.imshow(vs)
plt.plot(gt_path, 'w', linewidth=2.)
plt.plot(soln_path, 'g', linewidth=1.)
plt.xlim(0, np.size(vs,1)+1)
plt.ylim(0, np.size(vs,0)+1)
plt.savefig('out/soln.pdf')
