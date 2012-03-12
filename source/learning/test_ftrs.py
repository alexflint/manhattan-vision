import numpy as np
import py_indoor_context
import matplotlib.pyplot as plt

ftr_dir = '/home/alex/Code/indoor_context/data/svm_features/test'

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence('lab_kitchen1', [20,25])

inst0 = mgr.GetInstance(0)
inst1 = mgr.GetInstance(1)

fm = py_indoor_context.FeatureManager(ftr_dir)
fm.ComputeMockFeatures(inst0)
fm.CommitFeatures()
f2 = fm.GetFeature(2,0)

fm.ComputeMockFeatures(inst1)
fm.CommitFeatures()
assert(not np.all(fm.GetFeature(2,0) == f2))

fm.LoadFeaturesFor(inst0)
assert(np.all(fm.GetFeature(2,0) == f2))
