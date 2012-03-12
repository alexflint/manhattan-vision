import numpy as np
import py_indoor_context

FeatureStoreBase = '/home/alex/Code/indoor_context/data/svm_features'

class Datasets:
    All = [
        ('exeter_bursary', range(37)),
        ('exeter_mcr1', range(55)),
        ('lab_atrium2', range(21)),
        ('lab_foyer1', range(50)),
        ('lab_foyer2', range(51)),
        ('lab_ground1', range(56)),
        ('lab_kitchen1', range(93)),
        ('som_corr1', range(45)),
    ]

    class Small:
        TrainingSet = [
            ('lab_kitchen1', [25,35,45]),
        ]

        TestSet = [
            ('lab_kitchen1', [22,62]),
        ]

    class Large:
        TrainingSet = [
            ('lab_kitchen1', range(5,93,5)),
            ('exeter_mcr1', range(5,55,5)),
            ('lab_foyer1', range(5,50,5)),
            ('lab_foyer2', range(5,51,5)),
            ('som_corr1', range(5,45,5)),
            ('lab_ground1', range(5,51,5)),
        ]

        TestSet = [
            ('lab_kitchen1', range(2,93,10)),
            ('exeter_mcr1', range(2,55,10)),
            ('lab_foyer1', range(2,50,10)),
            ('lab_foyer2', range(2,51,10)),
            ('som_corr1', range(2,45,10)),
            ('lab_ground1', range(2,51,10)),
        ]

# Params for ICCV comparison
class ICCV:
    StereoOffsets = ([-1, 1])
    NumAuxFrames = len(StereoOffsets)
    StereoWeights = np.array([1./NumAuxFrames] * NumAuxFrames)
    Weights = np.hstack([.001, 100., 5., 50.*StereoWeights])
    CornerPenalty = 70.
    OcclusionPenalty = 50.
    Params = py_indoor_context.ManhattanHyperParameters(Weights,
                                                        CornerPenalty,
                                                        OcclusionPenalty)

# Params for ECCV2010 comparison
class ECCV2010:
    Weights = np.array([1.])
    CornerPenalty = 100.
    OcclusionPenalty = 0.
    Params = py_indoor_context.ManhattanHyperParameters(Weights,
                                                        CornerPenalty,
                                                        OcclusionPenalty)

class ECCV2010_v2:
    Weights = np.array([1.])
    CornerPenalty = 10.
    OcclusionPenalty = 0.
    Params = py_indoor_context.ManhattanHyperParameters(Weights,
                                                        CornerPenalty,
                                                        OcclusionPenalty)
