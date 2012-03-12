import os
import numpy as np

import py_indoor_context
import training_helpers
import training_params

np.set_printoptions(precision=3)

test_set = training_params.LargeTestSet

stereo_offsets = training_params.ICCV.StereoOffsets
params = training_params.ICCV.Params

output_path = 'experiments/feb29_iccvparams_depthmax'

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
mgr.ComputeAllFeatures(stereo_offsets, False)  # don't normalize for ICCV

# Run evaluation
reporter = training_helpers.Reporter([], instances, output_path)
reporter.generate_report(params, extended=True)
