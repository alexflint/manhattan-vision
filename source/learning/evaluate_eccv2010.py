import os
import numpy as np

import py_indoor_context
import training_helpers
import training_params

np.set_printoptions(precision=3)

test_set = training_params.Datasets.Large.TestSet

params = training_params.ECCV2010_v2.Params

feature_store = os.path.join(training_params.FeatureStoreBase, 'eccv2010')
output_path = 'experiments/mar05_eccv2010_cornerpenalty10'

# Check that we're not about to overwrite a previous experiment
if os.path.exists(output_path):
    print 'Error: Experiment dir already exists: ',output_path
    exit(-1)

# Load the dataset
mgr = py_indoor_context.TrainingManager()
instances = []
for sequence,frame_ids in test_set:
    print '  From %s loading frames %s' % (sequence, ','.join(map(str, frame_ids)))
    instances += training_helpers.load_sequence(mgr, sequence, frame_ids)

print 'Loaded %d instances' % len(instances)

# Compute features
fm = py_indoor_context.FeatureManager(feature_store)
for inst in instances:
    fm.ComputeSweepFeatures(inst)
    fm.CommitFeatures()

# Run evaluation
reporter = training_helpers.Reporter([], instances, fm, output_path)
reporter.generate_report(params, extended=True)
