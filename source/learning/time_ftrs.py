import math
import os
import training_params
import py_indoor_context

# Load a frame
mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence('lab_kitchen1', [22,32,42])

# Compute features
feature_store = os.path.join(training_params.FeatureStoreBase, 'manual')
fm = py_indoor_context.FeatureManager(feature_store)
fm.LoadFeaturesFor(mgr.GetInstance(0))
fm.LoadFeaturesFor(mgr.GetInstance(1))
fm.LoadFeaturesFor(mgr.GetInstance(2))
