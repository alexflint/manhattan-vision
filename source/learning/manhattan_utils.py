import numpy as np
import training_pb2 as proto

import proto_utils
import path

def flip_path(ys, affine):
    assert(np.shape(affine) == (2,))
    return [ int(round(y*affine[0]+affine[1])) for y in ys ]

def invert_affine(aff):
    return [1./aff[0], -1.*aff[1]/aff[0]]


class ManhattanProblem:
    def __init__(self, data):
        assert(type(data) is proto.ExampleFrame)
        self.data = data
        self.F = proto_utils.unpack_features(data.features)
        self.H = proto_utils.unpack_matrix(data.manhattan_homology)
        self.M = proto_utils.unpack_matrix(data.image_to_grid)
        self.M_grid = np.dot(self.M, np.dot(self.H, np.linalg.inv(self.M)))
        assert(abs(self.M_grid[1][0]) < 1e-6)
        assert(abs(self.M_grid[2][0]) < 1e-6)
        assert(abs(self.M_grid[2][1]) < 1e-6)
        assert(abs(1. - self.M_grid[2][2]) < 1e-6)

    # Get the affine floor to ceiling scaling parameters for y coords in the grid
    def get_manhattan_affine(self):
        return self.M_grid[1][1:]

class ManhattanSolution:
    def __init__(self, problem, ys, orients=None):
        assert isinstance(problem, ManhattanProblem), problem
        assert np.ndim(ys) == 1
        self.problem = problem
        self.ys = ys
        self.orients = orients
        self.pair = (ys,orients)
        self.losses = None    # lazily evaluated
        self.lossfunc = None  # lazily evaluated

    def compute_path_pair(self):
        yhorizon = self.problem.data.horizon_row
        aff = self.problem.get_manhattan_affine()  # floor -> ceiling
        affinv = invert_affine(aff)     # ceiling -> floor
        floor_ys = np.empty(len(self.ys))
        ceil_ys = np.empty(len(self.ys))
        for i,y in enumerate(self.ys):
            if y > yhorizon:
                floor_ys[i] = y
                ceil_ys[i] = 1.*y*aff[0] + aff[1]
            else:
                ceil_ys[i] = y
                floor_ys[i] = 1.*y*affinv[0] + affinv[1]
        return floor_ys,ceil_ys

    def compute_loss_terms(self, lossfunc):
        # TODO: change this so that equivalent floor/ceiling paths have identical loss
        if (self.losses is None or self.lossfunc != lossfunc):
            shape = self.problem.F.shape[:2]
            floor_ys,ceil_ys = self.compute_path_pair()
            floor_losses = path.compute_loss_terms(floor_ys, shape, lossfunc)
            ceil_losses = path.compute_loss_terms(ceil_ys, shape, lossfunc)
            ysplit = np.clip(self.problem.data.horizon_row, 0, shape[0])
            self.losses = np.vstack((ceil_losses[:ysplit], floor_losses[ysplit:]))
            self.lossfunc = lossfunc
        return self.losses

    # Compute loss for a hypothesized solution assuming we are the ground truth
    def compute_loss(self, hyp, lossfunc):
        # TODO: change this so that equivalent floor/ceiling paths have identical loss
        losses = self.compute_loss_terms(lossfunc)
        return np.sum(losses[[hyp.ys, range(len(hyp.ys))]])
    




# Compute the mean of each feature
def compute_feature_means(data):
    nf = np.size(data[0][0].F, 2)
    return np.array([np.mean([p.F[:,:,i] for p,s in data]) for i in range(nf)])

# Compute the variance of each feature
def compute_feature_variances(data):
    nf = np.size(data[0][0].F, 2)
    return np.array([np.var([p.F[:,:,i] for p,s in data]) for i in range(nf)])

# Centralize and normalize each feature
def center_data(instances):
    u = compute_feature_means(instances)
    s = compute_feature_variances(instances)
    for prob,soln in instances:
        prob.F -= u
        prob.F /= np.sqrt(s)

def prepare_svmstruct_data(dataset, center=True):
    # The mapping of orientations is highly imperfect
    instances = []
    for eg in dataset.examples:
        problem = ManhattanProblem(eg)
        soln = ManhattanSolution(problem,
                                 proto_utils.unpack_matrix(eg.gt_path),
                                 proto_utils.unpack_matrix(eg.gt_orients))
        instances.append((problem, soln))

    if center:
        center_data(instances)

    return instances
