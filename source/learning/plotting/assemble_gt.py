# Compute the average per-sequence performance

import sys
import py_indoor_context
import training_params
import training_helpers

dataset = training_params.Datasets.Large.TestSet
mgr = py_indoor_context.TrainingManager()
a,instances = training_helpers.load_dataset(mgr, [], dataset)

pattern = 'out/%s_frame%03d_gt.png'
for inst in instances:
    inst.OutputGroundTruthViz(pattern % (inst.GetSequenceName(),inst.GetFrameId()))
