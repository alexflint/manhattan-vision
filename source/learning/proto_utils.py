"""Helpers for protocol buffers in python"""

import numpy as np
import training_pb2 as proto

def unpack_matrix(m):
    # squeeze() removes unnecessary dimensions
    return np.squeeze(np.array(m.entries).reshape((m.rows,m.cols)))

def unpack_features(featureset):
    shape = (featureset[0].left.rows,
             featureset[0].left.cols,
             len(featureset))
    F = np.empty(shape)
    for i,feature in enumerate(featureset):
        F[:,:,i] = unpack_matrix(feature.left)
        if (feature.right.rows > 0):
            # just merge left and right features for now
            F[:,:,i] += unpack_matrix(feature.right)
    return F

def load_examples(path):
    with open(path, 'rb') as f:
        return proto.ExampleSet.FromString(f.read())

