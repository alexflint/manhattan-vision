function [w cases]=learn_manhattan_dp(cases)
% learn_manhattan_dp
%   Entry point to discriminative training for manhattan DP reconstructor

  if nargin==0
    cases = dp_load_cases('lab_kitchen1', 5:5:90, 'default');
  end

  randn('state', 0);
  rand('state', 0);

  % ------------------------------------------------------------------
  %                                                      Generate data
  % ------------------------------------------------------------------

  % patterns contains symbolic indices into an underlying training set
  assert(length(cases) >= 1);
  feature_size = size(cases(1).pixel_features, 3);
  psi_size = 3*feature_size + 2;  % +2 is for num_walls and num_occlusions
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
  w = model.w ;

  % ------------------------------------------------------------------
  %                                                         Evaluation
  % ------------------------------------------------------------------
  evaluate(cases, model.w);
  
  % ------------------------------------------------------------------
  %                                               SVM struct callbacks
  % ------------------------------------------------------------------

  function delta = lossCB(param, estimated, ground_truth)
	  disp('### Computing loss ###');
    delta = get_loss(estimated.orients, ground_truth.orients);
    assert(all(size(delta) == [1 1]));
  end

  function psi = featureCB(param, casedata, soln)
	  disp('### Computing psi ###');
		psi = sparse(get_psi(casedata.pixel_features, soln));
    assert(size(psi,2) == 1);
  end

  function soln = classifyCB(param, model, casedata)
	  disp('### Doing Inference ###');
    soln = reconstruct(casedata, model.w);
  end

  function contra_soln = marginCB(param, model, casedata, ground_truth)
  % find margin-rescaling largest violation
  % argmax_y delta(yi, y) + < psi(x,y), w >
	  disp('### Finding most-violated constraint ###');

    margin_calls = margin_calls + 1;
    if mod(margin_calls, length(cases)*5) == 0
      disp(['Called ' num2str(margin_calls) ' times so far']);
      cur_weights=model.w
      evaluate(cases, model.w);
    end
    
    obj = create_contra_objective(casedata.pixel_features, model.w, ground_truth);
    contra_soln = dp_solve(casedata.frame, obj);
  end
end
