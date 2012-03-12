"""Maximum margin training for the diagonal model. See diagonal.py."""

import os
import itertools
import numpy as np
import svmapi

import py_indoor_context
import training_helpers
import training_params

from auto_indent import AutoIndent

np.set_printoptions(precision=3)
AutoIndent.install()

class LossFunctions:
    L1=1
    DEPTH=2
    TWOLABELLING=3

class FeatureSets:
    ALLMULTIVIEW=1
    ALLMONO=10
    RGB=11
    RGBSWEEPS=12
    MOCK=20

############################################################
StereoOffsets = [-5, -1, 1, 5]

Dataset = training_params.Datasets.Large
FeatureSet = FeatureSets.ALLMULTIVIEW
LossFunction = LossFunctions.TWOLABELLING

#OutputPath = 'experiments/foo'
OutputPath = 'experiments/mar07_labellingloss_multiview-all_large'

FeatureStore = os.path.join(training_params.FeatureStoreBase, 'multiview-all')

############################################################

Mgr = py_indoor_context.TrainingManager()
FtrMgr = py_indoor_context.FeatureManager(FeatureStore)
Inference = py_indoor_context.ManhattanInference()
Reporter = None


def relerr(a,b):
    return np.abs(a-b)/b

def abserr(a,b):
    return np.abs(a-b)

def errcheck(a, b, reltol=1e-5, abstol=1e-6):
    return abserr(a,b) > abstol and relerr(a,b) > reltol

def configure_loss(instance, lossfunc, conditioning=0.):
    if lossfunc == LossFunctions.L1:
        instance.ConfigureL1Loss(conditioning)
    elif lossfunc == LossFunctions.DEPTH:
        instance.ConfigureDepthLoss(conditioning)
    elif lossfunc == LossFunctions.TWOLABELLING:
        instance.ConfigureLabellingLoss(conditioning)
    else:
        raise Exception('Unknown loss function '+str(lossfunc))

def compute_features(instance, ftrmgr, choice):
    if choice == FeatureSets.ALLMULTIVIEW:
        ftrmgr.ComputeMultiViewFeatures(instance, StereoOffsets)
    elif choice == FeatureSets.ALLMONO:
        ftrmgr.ComputeMonoFeatures(instance, 'all,-nbr_sweeps')
    elif choice == FeatureSets.RGB:
        ftrmgr.ComputeMonoFeatures(instance, 'rgb')
    elif choice == FeatureSets.RGBSWEEPS:
        ftrmgr.ComputeMonoFeatures(instance, 'rgb,sweeps')
    elif choice == FeatureSets.MOCK:
        ftrmgr.ComputeMockFeatures(instance)
    else:
        raise Exception('Unknown feature set: '+str(FeatureSet))

#
# Begin SVM-struct API
#
def read_examples(filename, sparm):
    """Reads and returns x,y example pairs from a file.
    
    This reads the examples contained at the file at path filename and
    returns them as a sequence.  Each element of the sequence should
    be an object 'e' where e[0] and e[1] is the pattern (x) and label
    (y) respectively.  Specifically, the intention is that the element
    be a two-element tuple containing an x-y pair."""

    # We actually ignore the filename passed to us

    print '\n'

    # Check that we're not about to overwrite a previous experiment
    if os.path.exists(OutputPath):
        print 'Error: Experiment dir already exists: ',OutputPath
        exit(-1)

    # Load the dataset
    training_instances,test_instances = \
        training_helpers.load_dataset(Mgr,
                                      Dataset.TrainingSet,
                                      Dataset.TestSet)
    assert(Mgr.NumInstances() == len(training_instances) + len(test_instances))

    # Compute features
    for inst in itertools.chain(training_instances, test_instances):
        compute_features(inst, FtrMgr, FeatureSet)
        FtrMgr.CommitFeatures()
    Mgr.NormalizeFeatures(FtrMgr)

    # Compute loss terms
    for inst in itertools.chain(training_instances, test_instances):
        configure_loss(inst, LossFunction)

    # Set up the reporter
    global Reporter
    Reporter = training_helpers.Reporter(training_instances,
                                         test_instances,
                                         FtrMgr,
                                         OutputPath)

    # Create a structure suitable for SVM-struct
    return training_helpers.prepare_svm_data(training_instances)

def init_model(sample, model, sparm):
    """Initializes the learning model.
    
    Initialize the structure model model.  The model.size_psi must be set to
    the number of features.  The ancillary purpose is to add any
    information to model that is necessary from the user code
    perspective.  This function returns nothing."""

    # The weights are not simple the length of the data features: they
    # also include terms for the transition matrix
    FtrMgr.LoadFeaturesFor(Mgr.GetInstance(0))   # ensure something is loaded
    model.size_psi = FtrMgr.NumFeatures() + 2

def init_constraints(sample, sm, sparm):
    """Initializes special constraints.

    Returns a sequence of initial constraints.  Each constraint in the
    returned sequence is itself a sequence with two items (the
    intention is to be a tuple).  The first item of the tuple is a
    document object.  The second item is a number, indicating that the
    inner product of the feature vector of the document object with
    the linear weights must be greater than or equal to the number
    (or, in the nonlinear case, the evaluation of the kernel on the
    feature vector with the current model must be greater).  This
    initializes the optimization problem by allowing the introduction
    of special constraints.  Typically no special constraints are
    necessary.  A typical constraint may be to ensure that all feature
    weights are positive.

    Note that the slack id must be set.  The slack IDs 1 through
    len(sample) (or just 1 in the combined constraint option) are used
    by the training examples in the sample, so do not use these if you
    do not intend to share slack with the constraints inferred from
    the training data.

    The default behavior is equivalent to returning an empty list,
    i.e., no constraints."""

    # Encode positivity constraints for the last two items of the weight vector
    constraints = []
    ftrlen = sm.size_psi
    for i in range(ftrlen-2, ftrlen):
        # Create a sparse vector which selects out a single feature.
        v = (np.arange(ftrlen) == i).astype(float)
        sparse = svmapi.Sparse(tuple(v))
        # The left hand side of the inequality is a document.
        # These ids leave a gap of size ftrlen, but I don't think this matters
        lhs = svmapi.Document([sparse], costfactor=1, slackid=len(sample)+i+1)
        # Append the lhs and the rhs
        constraints.append((lhs, 0))
    return constraints

def classify_example(instance, model, sparm):
    """Given a pattern x, return the predicted label."""

    params = training_helpers.create_params_from_psi(list(model.w))
    payoffs = py_indoor_context.DPPayoffs()
    FtrMgr.LoadFeaturesFor(instance)
    FtrMgr.CompileFeatures(params, payoffs)
    soln = Inference.Solve(instance, payoffs)
    return soln

def find_most_violated_constraint_slack(x, y, model, sparm):
    """Return ybar associated with x's most violated constraint.

    The find most violated constraint function for slack rescaling.
    The default behavior is that this returns the value from the
    general find_most_violated_constraint function."""

    raise Exception('Slack rescaling not implemented')

def find_most_violated_constraint_margin(instance, gt, model, sparm):
    """Return ybar associated with x's most violated constraint.

    The find most violated constraint function for margin rescaling.
    The default behavior is that this returns the value from the
    general find_most_violated_constraint function."""

    #print '\n\nFinding most violated constraint'

    assert(isinstance(instance, py_indoor_context.TrainingInstance))
    assert(isinstance(gt, py_indoor_context.ManhattanHypothesis))

    params = training_helpers.create_params_from_psi(list(model.w))

    aug_payoffs = py_indoor_context.DPPayoffs()
    reg_payoffs = py_indoor_context.DPPayoffs()

    FtrMgr.LoadFeaturesFor(instance)
    FtrMgr.CompileWithLoss(params, instance, aug_payoffs)
    FtrMgr.Compile(params, reg_payoffs)

    # Solve augmented problem
    aug_soln = Inference.Solve(instance, aug_payoffs)
    score = aug_payoffs.ComputeScore(aug_soln)

    # Compute loss on regular problem
    reg_score = reg_payoffs.ComputeScore(aug_soln)
    reg_loss = instance.ComputeLoss(aug_soln)

    # Check the score
    check_score = reg_score + reg_loss
    if errcheck(score, check_score):
        print '\n*** Inconsistent score!'
        print '  rel error:',relerr(score, check_score)
        print '  abs error:',abserr(score, check_score)
        print '  Aug score:',score
        print '  Reg score:',reg_score
        print '  Loss:',reg_loss
        print '  Reg score + loss:',check_score
        print '  Error:',abs(score-check_score)
        training_helpers.print_params(params)
        #exit(0)

    # check the score another way
    ftr = training_helpers.get_feature(FtrMgr, instance, aug_soln)
    reg_score2 = np.dot(list(ftr), list(model.w))
    reg_loss2 = loss(gt, aug_soln, sparm)
    if errcheck(reg_score, reg_score2):
        print '\n*** Inconsistent score!'
        print '  rel error:',relerr(reg_score, reg_score2)
        print '  abs error:',abserr(reg_score, reg_score2)
        print '  ftr <dot> soln:',reg_score2
        print '  payoffs.ComputeScore(soln):',reg_score
        print '  Instance: %s:%d' % (instance.GetSequenceName(),instance.GetFrameId())
        print '  ftr:',ftr
        print '  model.w:',list(model.w)
        training_helpers.print_params(params)
        #exit(0)

    # check the loss
    if errcheck(reg_loss, reg_loss2):
        print '\n*** Inconsistent loss!'
        print '  rel error:',relerr(reg_loss, reg_loss2)
        print '  abs error:',abserr(reg_loss, reg_loss2)
        print '  instance.ComputeLoss(soln):',reg_loss
        print '  loss(...):',reg_loss2
        training_helpers.print_params(params)
        #exit(0)

    # Compute GT score and check slack
    gt_score = reg_payoffs.ComputeScore(gt)
    margin = gt_score - reg_score  # this is the margin we're trying to maximize!
    if (margin > reg_loss):
        # The ground truth might not be in the hypothesis class
        # (e.g. when the GT path extends beyond the grid bounds), so
        # the most-violated inference might find a constraint that has
        # slack lower than that for the ground truth. The slack for
        # the ground truth is always zero, so if the slack for the
        # solution that the DP found is negative then we replace it
        # with the ground truth. One way to think about it is that our
        # hypothesis class is {all representable manhattan models} +
        # {ground truth}, which we perform inference in by comparing
        # the best representable manhattan model (as found by DP) to
        # the ground truth. The problem here is that at test time
        # we're performing inference in the hypothesis class {all
        # representable manhattan models}. I don't know what the
        # impact of training and testing on these subtly different
        # hypothesis classes is.
        aug_soln = gt
        print '\n+++Negative slack, replacing with gt (slack=%f)' % (reg_loss-margin)
        #print '  Margin:',margin
        #print '  Loss:',reg_loss
        #print '  Slack:',reg_loss-margin

    #print '\n\nFinding most violated constraint'
    #print '  data weights: ',params.GetWeights()
    #print '  corner penalty:',params.GetCornerPenalty()
    #print '  occlusion penalty:',params.GetOcclusionPenalty()
    #print '  feature(true): ',gt_ftr
    #print '  feature(aug-soln): ',aug_ftr
    #print '  score(aug-soln): ',np.dot(list(model.w), aug_ftr)
    #print '  loss(aug-soln): ',gt.GetInstance().ComputeLoss(aug_soln)

    return aug_soln

def psi(instance, hyp, model, sparm):
    """Return a feature vector representing pattern x and label y.

    This is the combined feature function, which this returns either a
    svmapi.Sparse object, or sequence of svmapi.Sparse objects (useful
    during kernel evaluations, as all components undergo kernel
    evaluation separately).  There is no default behavior."""

    assert(isinstance(instance, py_indoor_context.TrainingInstance))
    assert(isinstance(hyp, py_indoor_context.ManhattanHypothesis)), type(hyp)

    return svmapi.Sparse(training_helpers.get_feature(FtrMgr, instance, hyp))

def loss(gt, hyp, sparm):
    """Return the loss of ybar relative to the true labeling y.
    
    Returns the loss for the correct label y and the predicted label
    ybar.  In the event that y and ybar are identical loss must be 0.
    Presumably as y and ybar grow more and more dissimilar the
    returned value will increase from that point.  sparm.loss_function
    holds the loss function option specified on the command line via
    the -l option.

    The default behavior is to perform 0/1 loss based on the truth of
    y==ybar."""

    assert(isinstance(gt, py_indoor_context.ManhattanHypothesis)), type(gt)
    assert(isinstance(hyp, py_indoor_context.ManhattanHypothesis)), type(hyp)

    return gt.GetInstance().ComputeLoss(hyp)

def print_iteration_stats(ceps, cached_constraint, sample, model,
                          cset, alpha, sparm):
    """Called just before the end of each cutting plane iteration.

    This is called just before the end of each cutting plane
    iteration, primarily to print statistics.  The 'ceps' argument is
    how much the most violated constraint was violated by.  The
    'cached_constraint' argument is true if this constraint was
    constructed from the cache.
    
    The default behavior is that nothing is printed."""

    # generate a report
    params = training_helpers.create_params_from_psi(list(model.w))
    Reporter.add_iteration(params)

def print_learning_stats(sample, model, cset, alpha, sparm):
    """Print statistics once learning has finished.
    
    This is called after training primarily to compute and print any
    statistics regarding the learning (e.g., training error) of the
    model on the training sample.  You may also use it to make final
    changes to model before it is written out to a file.  For example, if
    you defined any non-pickle-able attributes in model, this is a good
    time to turn them into a pickle-able object before it is written
    out.  Also passed in is the set of constraints cset as a sequence
    of (left-hand-side, right-hand-side) two-element tuples, and an
    alpha of the same length holding the Lagrange multipliers for each
    constraint.

    The default behavior is that nothing is printed."""

    params = training_helpers.create_params_from_psi(list(model.w))
    Reporter.add_iteration(params)    # add a final iteration
    Reporter.generate_report(params, extended=True)

def print_testing_stats(sample, model, sparm, teststats):
    """Print statistics once classification has finished.
    
    This is called after all test predictions are made to allow the
    display of any summary statistics that have been accumulated in
    the teststats object through use of the eval_prediction function.

    The default behavior is that nothing is printed."""
    print teststats

def eval_prediction(exnum, (x, y), ypred, model, sparm, teststats):
    """Accumulate statistics about a single training example.
    
    Allows accumulated statistics regarding how well the predicted
    label ypred for pattern x matches the true label y.  The first
    time this function is called teststats is None.  This function's
    return value will be passed along to the next call to
    eval_prediction.  After all test predictions are made, the last
    value returned will be passed along to print_testing_stats.

    On the first call, that is, when exnum==0, teststats==None.  The
    default behavior is that the function does nothing."""
    #if exnum==0: teststats = []
    #print 'on example',exnum,'predicted',ypred,'where correct is',y
    #teststats.append(loss(y, ypred, sparm))
    #return teststats
    return None

def write_model(filename, model, sparm):
    """Dump the structmodel model to a file.
    
    Write the structmodel model to a file at path filename.

    The default behavior is equivalent to
    'cPickle.dump(model,bz2.BZ2File(filename,'w'))'."""
    import cPickle
    with open(filename, 'w') as fd:
        cPickle.dump(model, fd)

def read_model(filename, sparm):
    """Load the structure model from a file.
    
    Return the structmodel stored in the file at path filename, or
    None if the file could not be read for some reason.

    The default behavior is equivalent to
    'return cPickle.load(bz2.BZ2File(filename))'."""
    import cPickle
    with open(filename, 'r') as fd:
        return cPickle.load(fd)

def write_label(fileptr, y):
    """Write a predicted label to an open file.

    Called during classification, this function is called for every
    example in the input test file.  In the default behavior, the
    label is written to the already open fileptr.  (Note that this
    object is a file, not a string.  Attempts to close the file are
    ignored.)  The default behavior is equivalent to
    'print>>fileptr,y'"""
    print>>fileptr,y

def print_help():
    """Help printed for badly formed CL-arguments when learning.

    If this function is not implemented, the program prints the
    default SVM^struct help string as well as a note about the use of
    the --m option to load a Python module."""
    import svmapi
    print svmapi.default_help

def print_help_classify():
    """Help printed for badly formed CL-arguments when classifying.

    If this function is not implemented, the program prints the
    default SVM^struct help string as well as a note about the use of
    the --m option to load a Python module."""
