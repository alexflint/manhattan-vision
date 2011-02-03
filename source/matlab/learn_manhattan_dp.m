function [weights loaded_cases]=learn_manhattan_dp(cases)
% learn_manhattan_dp
%   Entry point to discriminative training for manhattan DP reconstructor
  
  % pick an output directory
  v = clock;
  basedir = '/home/alex/Code/indoor_context/source/matlab';
  outdir = sprintf('%s/runs/%04d-%02d-%02d %02d:%02d.%02d', basedir, round(v));
  r = mkdir(outdir);
  check r;
  
  % create symbolic links to this dir
  linkpath = [basedir '/lastrun'];
  if exist(linkpath)
    system(['rm ' linkpath]);
  end
  cmd = ['ln -s ''' outdir ''' ''' linkpath ''''];
  report cmd;
  system(cmd);

  % load cases
  if nargin==0
    cases = dp_load_cases('lab_kitchen1', 5:5:90, 'default');
    loaded_cases = cases;
  end

  randn('state', 0);
  rand('state', 0);

  % ------------------------------------------------------------------
  %                                                      Generate data
  % ------------------------------------------------------------------

  % patterns contains symbolic indices into an underlying training set
  check length(cases) >= 1;
  pix_ftr_size = size(cases(1).pixel_features, 3);
  wall_ftr_size = size(cases(1).wall_features, 3);
  psi_size = 3*pix_ftr_size + 2*wall_ftr_size + 2;  % +2 is for num_walls and num_occlusions
  patterns = cell(length(cases),1);
  labels = cell(length(cases),1);
  for i = 1:length(cases)
    patterns{i} = cases(i);
    labels{i} = cases(i).ground_truth;
  end

  % ------------------------------------------------------------------
  %                                                    Run SVM struct
  % ------------------------------------------------------------------

  sparm.patterns                 = patterns ;
  sparm.labels                   = labels ;
  sparm.lossFn                   = @lossCB ;
  %sparm.findMostViolatedSlackFn = @constraintCB ;
  sparm.findMostViolatedMarginFn = @marginCB ;
  sparm.psiFn                    = @featureCB ;
  sparm.sizePsi                  = psi_size;

  margin_calls = 0;
  
  % -v: verbosity for structSVM
  % -y: verbosity for svm_light
  % -b: percentage of training examples to renew examples for (as
  %     percentage in [0,100])
  % -f: number of constraints to cache
  % -o: type of rescaling (margin or slack)
  common_args = ' -o 2 -v 3 -y 1 ';
  basic_args = [common_args ' -c 0.00001'];
  caching_args = [common_args ' -c 0.01  -w 4  -f 5 '];
  svm_struct_args = basic_args;

  begin_learn_time = tic;
  last_eval_time = tic;
  last_accuracy = -1;
  
  model = svm_struct_learn(svm_struct_args, sparm) ;

  % ------------------------------------------------------------------
  %                                                         Evaluation
  % ------------------------------------------------------------------
  weights = unpack_weights(model.w', pix_ftr_size, wall_ftr_size);
  evaluate(cases, weights);
  
  % ------------------------------------------------------------------
  %                                               SVM struct callbacks
  % ------------------------------------------------------------------

  function delta = lossCB(param, estimated, ground_truth)
	  disp('### Computing loss ###');
    delta = get_pixel_loss(estimated, ground_truth);
    %check prod(size(delta)) == 1;
    disp('# Done. #')
  end

  function psi = featureCB(param, casedata, soln)
	  disp('### Computing psi ###');
		psi = sparse(get_psi(casedata.pixel_features, casedata.wall_features, soln));
    check size(psi,2) == 1;
    disp('# Done. #')
  end

  function soln = classifyCB(param, model, casedata)
	  disp('### Doing Inference ###');
    weights = unpack_weights(model.w', pix_ftr_size, wall_ftr_size);
    soln = reconstruct(casedata, weights);
    disp('# Done. #')
  end

  function contra_soln = marginCB(param, model, casedata, ground_truth)
  % find margin-rescaling largest violation
  % argmax_y delta(yi, y) + < psi(x,y), w >
	  disp('### Finding most-violated constraint ###');
    disp(['                                      Last Accuracy: ' ...
          num2str(last_accuracy) '%']);

    weights = unpack_weights(model.w', pix_ftr_size, wall_ftr_size);

    margin_calls = margin_calls + 1;
    iter = floor(margin_calls/length(cases));
    elapsed_time = toc(begin_learn_time);
    if (mod(margin_calls, 10) == 0)
      % report time
      disp(' ');
      disp(['Averaging ' num2str(1000*elapsed_time/margin_calls) ...
            'ms per call to marginCB']);
      disp(' ');
    end
    
    time_since_eval = toc(last_eval_time);
    %if mod(margin_calls, length(cases)*5) == 0
    if time_since_eval > 60   % evaluate solution once per minute
      outfile = sprintf('%s/iter%04d.mat', outdir, iter);
      save(outfile, 'weights', 'svm_struct_args', 'iter');
      
      disp(['Called ' num2str(margin_calls) ' times so far']);
      last_accuracy = evaluate(cases, weights);
      last_eval_time = tic;
    end
    
    obj = make_contra_objective(casedata.pixel_features, ...
                                casedata.wall_features, ...
                                weights, ...
                                ground_truth);
    contra_soln = dp_solve(casedata.frame, obj);
    
    disp('# Done. #')
  end
end

