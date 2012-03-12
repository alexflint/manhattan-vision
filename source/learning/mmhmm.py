"""Maximum margin HMM training."""

import svmapi
import numpy as np

import viterbi
import path

def read_examples(filename, sparm):
    """Reads and returns x,y example pairs from a file.
    
    This reads the examples contained at the file at path filename and
    returns them as a sequence.  Each element of the sequence should
    be an object 'e' where e[0] and e[1] is the pattern (x) and label
    (y) respectively.  Specifically, the intention is that the element
    be a two-element tuple containing an x-y pair."""

    return viterbi.generate_training_data(nexamples=30,
                                          noutliers=5,
                                          ntimesteps=10,
                                          nstates=20)

def init_model(sample, model, sparm):
    """Initializes the learning model.
    
    Initialize the structure model model.  The model.size_psi must be set to
    the number of features.  The ancillary purpose is to add any
    information to model that is necessary from the user code
    perspective.  This function returns nothing."""

    model.size_psi = sample[0][0].shape[2]

def classify_example(x, model, sparm):
    """Given a pattern x, return the predicted label."""

    w = list(model.w)
    return viterbi.solve(path.compute_data_terms(x, w))

def find_most_violated_constraint_slack(x, y, model, sparm):
    """Return ybar associated with x's most violated constraint.

    The find most violated constraint function for slack rescaling.
    The default behavior is that this returns the value from the
    general find_most_violated_constraint function."""

    print '*** Slack rescaling not implemented ***'

    raise Exception('Slack rescaling not implemented')

def find_most_violated_constraint_margin(x, y, model, sparm):
    """Return ybar associated with x's most violated constraint.

    The find most violated constraint function for margin rescaling.
    The default behavior is that this returns the value from the
    general find_most_violated_constraint function."""

    w = list(model.w)

    print '\nFinding most violated constraint'
    print '  w: ',w
    print '  y: ',y

    A = path.compute_loss_augmented_terms(x, w, y, path.L2)
    ybar = viterbi.solve(A)

    D = path.compute_data_terms(x, w)
    print '  ybar: ',ybar
    print '  loss: ',path.compute_loss(y, ybar, path.L2)
    #print 'Data terms:\n', np.round(D, 2)
    #print 'Loss augmented terms:\n', np.round(A, 2)

    return ybar

def psi(x, y, model, sparm):
    """Return a feature vector representing pattern x and label y.

    This is the combined feature function, which this returns either a
    svmapi.Sparse object, or sequence of svmapi.Sparse objects (useful
    during kernel evaluations, as all components undergo kernel
    evaluation separately).  There is no default behavior."""

    return svmapi.Sparse(path.sum_path_features(x, y))

def loss(y, ybar, sparm):
    """Return the loss of ybar relative to the true labeling y.
    
    Returns the loss for the correct label y and the predicted label
    ybar.  In the event that y and ybar are identical loss must be 0.
    Presumably as y and ybar grow more and more dissimilar the
    returned value will increase from that point.  sparm.loss_function
    holds the loss function option specified on the command line via
    the -l option.

    The default behavior is to perform 0/1 loss based on the truth of
    y==ybar."""
    return path.compute_loss(y, ybar, path.L2)

def print_iteration_stats(ceps, cached_constraint, sample, model,
                          cset, alpha, sparm):
    """Called just before the end of each cutting plane iteration.

    This is called just before the end of each cutting plane
    iteration, primarily to print statistics.  The 'ceps' argument is
    how much the most violated constraint was violated by.  The
    'cached_constraint' argument is true if this constraint was
    constructed from the cache.
    
    The default behavior is that nothing is printed."""
    print 'Current model is: ',list(model.w)

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

    w = list(model.w)

    print 'Model learned: ',w
    print 'Losses:',
    print [loss(y, classify_example(x, model, sparm), sparm) for x,y in sample]

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
    if exnum==0: teststats = []
    print 'on example',exnum,'predicted',ypred,'where correct is',y
    teststats.append(loss(y, ypred, sparm))
    return teststats

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
