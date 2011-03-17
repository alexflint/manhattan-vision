clear

% this parameter is now fixed at 1
%init_corner_penalty = 400;
%min_corner_penalty = 0;
%max_corner_penalty = 1e+4;

init_occlusion_penalty = 1;
min_occlusion_penalty = 0;
max_occlusion_penalty = 1e+3;

init_mono_weight = .001;
min_mono_weight = 1e-5;
max_mono_weight = 1;

init_stereo_weight = 50;
min_stereo_weight = 1e-3;
max_stereo_weight = 1e+3;

init_agreement_weight = 1000;
min_agreement_weight = 1e-3;
max_agreement_weight = 1e+3;

init_occlusion_weight = 40;
min_occlusion_weight = 1e-3;
max_occlusion_weight = 1e+3;
 
x0 = [ %init_corner_penalty ...
       init_occlusion_penalty ...
       init_mono_weight ...
       init_stereo_weight ...
       init_agreement_weight ...
       init_occlusion_weight ...
       ];

lower_bound = [ %min_corner_penalty ...
                min_occlusion_penalty ...
                min_mono_weight ...
                min_stereo_weight ...
                min_agreement_weight ...
                min_occlusion_weight ...
                ];
 
upper_bound = [ %max_corner_penalty ...
                max_occlusion_penalty ...
                max_mono_weight ...
                max_stereo_weight ...
                max_agreement_weight ...
                max_occlusion_weight ...
                ];
   
% precompute payoffs
evaluate_manhattan_dp(x0, 1);

Problem.f = @(x) evaluate_manhattan_dp(x');

opts.maxevals = 1000;
opts.showits = true;
bounds = [lower_bound; upper_bound]';

[y_min, x_min] = Direct(Problem, bounds, opts);
x_min = x_min';
