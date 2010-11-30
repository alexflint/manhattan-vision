function [weights cases]=learn_manhattan_dp(cases)
% learn_manhattan_dp
%   Entry point to discriminative training for manhattan DP reconstructor
  
  % pick an output directory
  v = clock;
  basedir = '/home/alex/Code/indoor_context/source/matlab';
  outdir = sprintf('%s/runs/%d:%d.%d %d-%d-%d', basedir, round(v([4 5 6 3 2 1])));
  r = mkdir(outdir);
  check r;
  
  % create symbolic links to this dir
  linkpath = [outdir '/lastrun'];
  system(['rm ' linkpath]);
  system(['ln -s ' outdir ' ' linkpath]);

  % load cases
  if nargin==0
    cases = dp_load_cases('lab_kitchen1', 5:5:90, 'default');
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
  
  model = svm_struct_learn(' -c 1.0 -o 2 -v 1 ', sparm) ;

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
    check prod(size(delta)) == 1;
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

    weights = unpack_weights(model.w', pix_ftr_size, wall_ftr_size);

    margin_calls = margin_calls + 1;
    if mod(margin_calls, length(cases)*5) == 0
      outfile = sprintf('%s/iter%04d.mat', outdir, margin_calls);
      save(outfile, 'weights');
      
      disp(['Called ' num2str(margin_calls) ' times so far']);
      disp(['Called ' num2str(margin_calls) ' times so far']);
      evaluate(cases, weights);
    end
    
    obj = make_contra_objective(casedata.pixel_features, ...
                                casedata.wall_features, ...
                                weights, ...
                                ground_truth);
    contra_soln = dp_solve(casedata.frame, obj);
    
    disp('# Done. #')
  end
end

