function [x0 lower_bound upper_bound] = manhattan_optimization_bounds()

%
% Note: lower bounds must all be >= 0 as we use a logarithmic
% parameterization
%

<<<<<<< .mine
init_occlusion_penalty = .8;
=======
init_corner_penalty = 1;
min_corner_penalty = 1e-3;
max_corner_penalty = 1e+3;

init_occlusion_penalty = 1;
>>>>>>> .r112
min_occlusion_penalty = 1e-3;
max_occlusion_penalty = 1e+3;

<<<<<<< .mine
init_mono_weight = .0001;
min_mono_weight = 1e-5;
=======
init_mono_weight = .00001;
min_mono_weight = 1e-7;
>>>>>>> .r112
max_mono_weight = 1;

<<<<<<< .mine
init_stereo_weight = .5;
=======
init_stereo_weight = .2;
>>>>>>> .r112
min_stereo_weight = 1e-3;
max_stereo_weight = 1e+3;

<<<<<<< .mine
init_agreement_weight = 10;
=======
init_agreement_weight = 2;
>>>>>>> .r112
min_agreement_weight = 1e-3;
max_agreement_weight = 1e+3;

<<<<<<< .mine
init_occlusion_weight = 1;
=======
init_occlusion_weight = .8;
>>>>>>> .r112
min_occlusion_weight = 1e-3;
max_occlusion_weight = 1e+3;
 
x0 = [ init_corner_penalty ...
       init_occlusion_penalty ...
       init_mono_weight ...
       init_stereo_weight ...
       init_agreement_weight ...
       init_occlusion_weight ...
       ];
   
lower_bound = [ min_corner_penalty ...
                min_occlusion_penalty ...
                min_mono_weight ...
                min_stereo_weight ...
                min_agreement_weight ...
                min_occlusion_weight ...
                ];
 
upper_bound = [ max_corner_penalty ...
                max_occlusion_penalty ...
                max_mono_weight ...
                max_stereo_weight ...
                max_agreement_weight ...
                max_occlusion_weight ...
                ];
            
% Optimum found by GP, march 21
assignin('caller', 'x_gp_mar21', exp([0 4.0038 -6.1209 1.2940 6.9078 -6.2493]));

% Parameters used for ICCV 2011
assignin('caller', 'x_iccv', [ 400 300 0.001 50 1000 40 ]);
