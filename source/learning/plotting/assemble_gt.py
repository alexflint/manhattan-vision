# Compute the average per-sequence performance

import sys
import py_indoor_context

if len(sys.argv) != 3:
    print 'Usage:',sys.argv[0],' SEQUENCE FRAMEID'
    exit(-1)

seq = sys.argv[1]
idx = int(sys.argv[2])
pattern = 'out/%s_frame%03d_gt.png'

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence(seq, [idx])

inst = mgr.GetInstance(0)
inst.OutputGroundTruthViz(pattern % (seq,idx))
