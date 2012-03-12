import os
import cPickle
import csv
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import py_indoor_context

kPenaltyFactor = 1000

def create_params_from_psi(w):
    # If this changes then we also need to update
    #   - init_constraints()  so that the right weights are positive
    #   - TrainingInstance::GetFeatureForHypothesis()
    wts = np.array(w[:-2], float)
    # The factor of 100 here _must_ match that in ComputeFeatureForHypothesis()
    return py_indoor_context.ManhattanHyperParameters(wts,
                                                      w[-2]*kPenaltyFactor,
                                                      w[-1]*kPenaltyFactor)

def get_psi_from_params(params):
    # If this changes then we also need to update
    #   - init_constraints()  so that the right weights are positive
    #   - TrainingInstance::GetFeatureForHypothesis()
    return np.hstack([ params.GetWeights(),
                       params.GetCornerPenalty() / kPenaltyFactor,
                       params.GetOcclusionPenalty() / kPenaltyFactor ])

def get_feature(ftrmgr, instance, hyp):
    ftrmgr.LoadFeaturesFor(instance)
    ftr = ftrmgr.ComputeFeatureForHypothesis(hyp)
    ftr[-2:] *= kPenaltyFactor
    return ftr

def compute_label_error(gt_labels, hyp_labels):
    assert gt_labels.shape == hyp_labels.shape
    return 1. * np.sum(gt_labels != hyp_labels) / gt_labels.size

def compute_twolabel_error(gt_labels, hyp_labels):
    assert gt_labels.shape == hyp_labels.shape
    return 1. * np.sum(np.clip(gt_labels,1,2) != np.clip(hyp_labels,1,2)) / gt_labels.size

def compute_depth_error(gt_depths, hyp_depths):
    assert gt_depths.shape == hyp_depths.shape
    return np.sum(np.abs(hyp_depths-gt_depths) / gt_depths) / gt_depths.size

def compute_depthmax_error(gt_depths, hyp_depths):
    assert gt_depths.shape == hyp_depths.shape
    return np.sum(np.abs(hyp_depths-gt_depths) / np.maximum(gt_depths,hyp_depths)) / gt_depths.size

def prepare_svm_data(instances):
    return [ (inst, inst.GetGroundTruth()) for inst in instances ]

def params_to_dict(params):
    return { 'weights':params.GetWeights(),
             'corner_penalty':params.GetCornerPenalty(),
             'occlusion_penalty':params.GetOcclusionPenalty()
             }

# Extract n items from xs, evenly spaced through the list
def linsample(xs, n):
    if len(xs) < n:
        return xs
    else:
        return [ xs[i] for i in np.linspace(0, len(xs)-1, n).round().astype(int) ]

# Load a sequence and return the loaded frames as a list
# TODO: move this to c++
def load_sequence(mgr, sequence, frame_ids):
    n = mgr.NumInstances()
    mgr.LoadSequence(sequence, frame_ids)
    return [mgr.GetInstance(i) for i in range(n, mgr.NumInstances())]

def load_dataset(mgr, training_set, test_set):
    training_instances = []
    print 'Loading training set'
    for sequence,frame_ids in training_set:
        print '  From %s loading frames %s' % (sequence, ','.join(map(str, frame_ids)))
        training_instances += load_sequence(mgr, sequence, frame_ids)

    print 'Loading test set'
    test_instances = []
    for sequence,frame_ids in test_set:
        print '  From %s loading frames %s' % (sequence, ','.join(map(str, frame_ids)))
        test_instances += load_sequence(mgr, sequence, frame_ids)

    return (training_instances, test_instances)

def print_params(params):
    print '  Current model:'
    print '    corner penalty:',params.GetCornerPenalty()
    print '    occlusion penalty:',params.GetOcclusionPenalty()
    print '    data weights:\n',params.GetWeights()

class Reporter:
    def __init__(self, training_instances, holdout_instances, ftrmgr, path):
        self.iteration = 0
        self.path = path
        self.ftrmgr = ftrmgr
        self.inference = py_indoor_context.ManhattanInference()

        self.training_instances = training_instances
        self.holdout_instances = holdout_instances

        self.psis = []
        self.training_stats = []
        self.holdout_stats = []

        self.parameters_file = os.path.join(path, 'parameters.csv')

        self.training_perf_file = os.path.join(path, 'training_performance.csv')
        self.holdout_perf_file = os.path.join(path, 'holdout_performance.csv')

        self.training_data_file = os.path.join(path, 'training_performance.pickled')
        self.holdout_data_file = os.path.join(path, 'holdout_performance.pickled')

        self.training_results_file = os.path.join(path, 'training_performance.pdf')
        self.holdout_results_file = os.path.join(path, 'holdout_performance.pdf')

        self.training_iters_file = os.path.join(path, 'training_iterations.csv')
        self.holdout_iters_file = os.path.join(path, 'holdout_iterations.csv')

        # Not used currently:
        #self.training_summary_file = os.path.join(path, 'training_summary.pdf')
        #self.holdout_summary_file = os.path.join(path, 'holdout_summary.pdf')

        self.viz_dir = os.path.join(path, 'out')
        if not os.path.isdir(path):
            assert(not os.path.exists(path))
            parentdir = os.path.dirname(os.path.abspath(path))
            assert(os.path.exists(parentdir))
            os.mkdir(path)
            assert(os.path.exists(path))
        if not os.path.isdir(self.viz_dir):
            os.mkdir(self.viz_dir)
            assert(os.path.exists(self.viz_dir))

    def draw_labels(self, hyp_labels, gt_labels, instance_name):
        plt.clf();
        plt.subplot(121)
        plt.title('Estimated %s' % instance_name)
        plt.imshow(hyp_labels.astype(float))
        plt.xlim(0, np.size(hyp_labels,1)+1)
        plt.ylim(0, np.size(hyp_labels,0)+1)
        plt.subplot(122)
        plt.title('True %s' % instance_name)
        plt.imshow(gt_labels.astype(float))
        plt.xlim(0, np.size(gt_labels,1)+1)
        plt.ylim(0, np.size(gt_labels,0)+1)

    def draw_payoffs(self, payoffs, hyp_path, gt_path, instance_name):
        vs = payoffs.Get(0) + payoffs.Get(1)
        plt.clf()
        plt.title('Example %s' % instance_name)
        plt.imshow(vs)
        plt.plot(gt_path, 'w', linewidth=2.)
        plt.plot(hyp_path, 'g', linewidth=1.)
        plt.xlim(0, np.size(vs,1)+1)
        plt.ylim(0, np.size(vs,0)+1)

    def plot_error_hist(self, errs):
        plt.clf()
        plt.hist(errs*100., bins=10, range=(0,50))
        plt.xlabel('Error magnitude')
        plt.ylabel('Frequency')
        plt.xlim(xmin=0, xmax=100)

    def plot_error_scatter(self, label_errs, depth_errs):
        plt.clf()
        plt.title('Depth error versus labelling error')
        plt.scatter(label_errs*100., depth_errs*100.)
        plt.xlabel('Labelling error')
        plt.ylabel('Depth error')
        plt.xlim(xmin=0, xmax=100)
        plt.ylim(ymin=0, ymax=100)

    def plot_param_evolution(self, psis, component_names):
        assert(len(self.training_instances) > 0)
        assert(len(psis[0]) == len(component_names))

        plt.clf()
        plt.subplot(121)  # make space for external legend - important!
        for i in range(np.size(psis, 1)):
            plt.plot(psis[:,i])
        plt.xlabel('Training iteration')
        plt.ylabel('Value of component')
        plt.legend(component_names,
                   bbox_to_anchor=(1.05,1),        # Place the legend outside the axes
                   loc=2,
                   borderaxespad=0.,
                   shadow=True)
        
    def plot_error_evolution(self, errors):
        plt.clf()
        plt.plot(errors*100.)
        plt.ylim(ymin=0, ymax=60)
        plt.xlabel('Training iteration')

    def compute_performance(self, instances, params):
        depth_errs = []
        depthmax_errs = []
        label_errs = []
        twolabel_errs = []

        for instance in instances:
            gt = instance.GetGroundTruth()
            gt_labels = instance.GetGroundTruthLabels()
            gt_depths = instance.GetGroundTruthDepths()

            payoffs = py_indoor_context.DPPayoffs()

            self.ftrmgr.LoadFeaturesFor(instance)
            self.ftrmgr.Compile(params, payoffs)

            hyp = self.inference.Solve(instance, payoffs)
            hyp_labels = self.inference.GetSolutionLabels()
            hyp_depths = self.inference.GetSolutionDepths()

            depth_errs.append(compute_depth_error(gt_depths, hyp_depths))
            depthmax_errs.append(compute_depthmax_error(gt_depths, hyp_depths))
            label_errs.append(compute_label_error(gt_labels, hyp_labels))
            twolabel_errs.append(compute_twolabel_error(gt_labels, hyp_labels))

        return ( np.mean(depth_errs),
                 np.mean(depthmax_errs),
                 np.mean(label_errs),
                 np.mean(twolabel_errs) )

    def report_performance(self, errors):
        depth_err,depthmax_err,label_err,twolabel_err = errors
        print '    {:<30}{:.1f}%'.format('Av. depth error:', depth_err*100.)
        print '    {:<30}{:.1f}%'.format('Av. depthmax error:', depthmax_err*100.)
        print '    {:<30}{:.1f}%'.format('Av. labelling error:', label_err*100.)
        print '    {:<30}{:.1f}%'.format('Av. 2-labelling error:', twolabel_err*100.)

    def add_iteration(self, params):
        print '*** TRAINING PHASE ',self.iteration

        assert(len(self.training_instances) > 0)
        assert(len(self.holdout_instances) > 0)

        # Report current parameters
        print_params(params)
        self.psis.append(np.hstack([ params.GetWeights(),
                                     params.GetCornerPenalty(),
                                     params.GetOcclusionPenalty() ]))

        # Evaluate on a sample from the training set
        sample = linsample(self.training_instances, 5)
        if self.iteration % 10 == 0:
            print '  Training set performance:'
            perf = self.compute_performance(sample, params)
            self.report_performance(perf)
            self.training_stats.append(perf)
        else:
            self.training_stats.append(-np.ones(4))

        # Evaluate on a sample from the holdout set
        sample = linsample(self.holdout_instances, 5)
        if self.iteration % 10 == 0:
            print '  Holdout set performance:'
            perf = self.compute_performance(sample, params)
            self.report_performance(perf)
            self.holdout_stats.append(perf)
        else:
            self.holdout_stats.append(-np.ones(4))

        self.iteration += 1

    def generate_report(self, params, extended=False):
        if len(self.training_instances) > 0:
            print '\n\nEvaluating on training set'
            self.generate_report_for(params,
                                     self.training_instances,
                                     self.psis,
                                     self.training_stats,
                                     self.training_results_file,
                                     self.training_data_file,
                                     self.training_perf_file,
                                     self.training_iters_file,
                                     extended)

        if len(self.holdout_instances) > 0:
            print '\n\nEvaluating on holdout set'
            self.generate_report_for(params,
                                     self.holdout_instances,
                                     self.psis,
                                     self.holdout_stats,
                                     self.holdout_results_file,
                                     self.holdout_data_file,
                                     self.holdout_perf_file,
                                     self.holdout_iters_file,
                                     extended)

    def generate_report_for(self,
                            params,
                            instances,
                            psis,          # params for each iteration
                            stats,         # array of errors for each iteration
                            report_file,   # .pdf with lots of figures
                            data_file,     # .pickle with lots of python data
                            perf_file,     # .csv with performance summary per instance
                            iters_file,    # .csv with performance at each iteration
                            extended=False):
        assert(len(instances) > 0)
        assert(len(psis) == len(stats))

        print_params(params)

        self.ftrmgr.LoadFeaturesFor(instances[0])  # ensure that something is loaded
        ftr_comments = [ self.ftrmgr.GetFeatureComment(i)
                         for i in range(self.ftrmgr.NumFeatures()) ]
        component_names = ftr_comments + ['Corner penalty', 'Occlusion penalty']

        param_history = [ params_to_dict(create_params_from_psi(psi))
                          for psi in psis ]

        topickle = {}  # to be pickle'd to a file at the end
        topickle['params'] = params_to_dict(params)
        topickle['features'] = ftr_comments
        topickle['results'] = {}
        topickle['params_history'] = param_history
        topickle['performance_history'] = stats

        depth_errs = []
        depthmax_errs = []
        label_errs = []
        twolabel_errs = []

        pdf = PdfPages(report_file)
        for instance in instances:
            instance_name = '%s:%s' % (instance.GetSequenceName(),
                                       instance.GetFrameId())

            # Get ground truth
            gt = instance.GetGroundTruth()
            gt_labels = instance.GetGroundTruthLabels()
            gt_depths = instance.GetGroundTruthDepths()
            gt_path = gt.GetPath()

            # Compute a payoff matrix
            payoffs = py_indoor_context.DPPayoffs()
            self.ftrmgr.LoadFeaturesFor(instance)
            self.ftrmgr.Compile(params, payoffs)

            # Run the inference algorithm
            hyp = self.inference.Solve(instance, payoffs)
            hyp_labels = self.inference.GetSolutionLabels()
            hyp_depths = self.inference.GetSolutionDepths()
            hyp_path = hyp.GetPath()

            # Compute scores (proportional to posterior)
            hyp_score = payoffs.ComputeScore(hyp)
            gt_score = payoffs.ComputeScore(gt)

            # Compute errors
            depth_err = compute_depth_error(gt_depths, hyp_depths)
            depthmax_err = compute_depthmax_error(gt_depths, hyp_depths)
            label_err = compute_label_error(gt_labels, hyp_labels)
            twolabel_err = compute_twolabel_error(gt_labels, hyp_labels)

            depth_errs.append(depth_err)
            depthmax_errs.append(depthmax_err)
            label_errs.append(label_err)
            twolabel_errs.append(twolabel_err)

            # Add to record to be pickled
            record = { 'sequence': instance.GetSequenceName(),
                       'frame_id': instance.GetFrameId(),
                       'estimated_path':hyp_path,
                       'estimated_path_orients':hyp.GetOrients(),
                       'estimated_score':hyp_score,
                       'gt_path':gt_path,
                       'gt_path_orients':gt.GetOrients(),
                       'gt_score':gt_score,
                       'depth_error':depth_err,
                       'depthmax_error':depthmax_err,
                       'labelling_error':label_err,
                       'twolabelling_error':twolabel_err,
                       }
            topickle['results'][instance_name] = record

            if extended:
                # Draw the solution as a PNG
                filename = ('%s_frame%03d_soln.png' % (instance.GetSequenceName(),
                                                       instance.GetFrameId()))
                vizfile = os.path.join(self.viz_dir, filename)
                self.inference.OutputSolutionViz(vizfile)

                # Plot the payoffs and paths
                self.draw_payoffs(payoffs, hyp_path, gt_path, instance_name)
                pdf.savefig()
                
                # Plot labels in grid coords
                self.draw_labels(hyp_labels, gt_labels, instance_name)
                pdf.savefig()

        depth_errs = np.array(depth_errs)
        depthmax_errs = np.array(depthmax_errs)
        label_errs = np.array(label_errs)
        twolabel_errs = np.array(twolabel_errs)

        # Plot the distribution of errors
        self.plot_error_hist(depth_errs)
        plt.title('Distribution of depth errors')
        pdf.savefig()

        # Plot the distribution of errors
        self.plot_error_hist(depthmax_errs)
        plt.title('Distribution of depth-max errors')
        pdf.savefig()

        self.plot_error_hist(label_errs)
        plt.title('Distribution of labelling errors')
        pdf.savefig()

        self.plot_error_hist(twolabel_errs)
        plt.title('Distribution of two-labelling errors')
        pdf.savefig()

        self.plot_error_scatter(label_errs, depth_errs)
        pdf.savefig()

        # Write per-instance performance to CSV
        with open(perf_file, 'w') as f:
            w = csv.writer(f)
            w.writerow([ 'Instance',
                         'Depth Error',
                         'Depth-max Error',
                         'Labelling Error',
                         'Two-Labelling Error' ])
            for instance_name in topickle['results']:
                r = topickle['results'][instance_name]
                w.writerow([ instance_name,
                             r['depth_error'],
                             r['depthmax_error'],
                             r['labelling_error'],
                             r['twolabelling_error'] ])

        # Compute final performance summary
        # Do not append these to self.stats because that array is full of
        # evaluations on a *sample* of the instances
        av_depth_err = np.mean(depth_errs)
        av_depthmax_err = np.mean(depthmax_errs)
        av_label_err = np.mean(label_errs)
        av_twolabel_err = np.mean(twolabel_errs)

        # Save performance summary to be pickled
        topickle['average_depth_error'] = av_depth_err
        topickle['average_depthmax_error'] = av_depthmax_err
        topickle['average_labelling_error'] = av_label_err
        topickle['average_twolabelling_error'] = av_twolabel_err

        # Print performance summary to output
        print 'Depth errors: ',depth_errs*100.
        print 'Depth-max errors: ',depthmax_errs*100.
        print 'Labelling errors: ',label_errs*100.
        print '2-Labelling errors: ',twolabel_errs*100.

        print '{:<30}{:.1f}%'.format('Av. depth error:', av_depth_err*100.)
        print '{:<30}{:.1f}%'.format('Av. depth-max error:', av_depthmax_err*100.)
        print '{:<30}{:.1f}%'.format('Av. labelling error:', av_label_err*100.)
        print '{:<30}{:.1f}%'.format('Av. 2-labelling error:', av_twolabel_err*100.)

        # Generate per-iteration report
        if len(stats) > 0:
            stats = np.array(stats)
            iter_depth_errs = stats[:,0]
            iter_depthmax_errs = stats[:,1]
            iter_label_errs = stats[:,2]
            iter_twolabel_errs = stats[:,3]

            # Plot evolution of parameters over time
            self.plot_param_evolution(np.asarray(psis), component_names)
            pdf.savefig()
            
            # Plot error versus iteration
            self.plot_error_evolution(iter_depth_errs)
            plt.ylabel('Average depth error (%)')
            pdf.savefig()

            self.plot_error_evolution(iter_depthmax_errs)
            plt.ylabel('Average depth-max error (%)')
            pdf.savefig()

            self.plot_error_evolution(iter_label_errs)
            plt.ylabel('Average labelling error (%)')
            pdf.savefig()

            self.plot_error_evolution(iter_twolabel_errs)
            plt.ylabel('Average two-labelling error (%)')
            pdf.savefig()

            # Write iteration details to csv
            with open(iters_file, 'w') as f:
                w = csv.writer(f)
                w.writerow([ 'Iteration',
                             'Mean depth error',
                             'Mean depth-max error',
                             'Mean labelling error',
                             'Mean two-labelling error' ] + component_names)
                for itr,(stat,psi) in enumerate(zip(stats,psis)):
                    assert(len(stat) == 4)
                    assert(len(psi) == len(component_names))
                    w.writerow([itr] + list(stat) + list(psi))

        # Save the pickle data
        with open(data_file, 'w') as f:
            cPickle.dump(topickle, f)

        # Write the PDF
        pdf.close()
