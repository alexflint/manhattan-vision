function [ y output ] = evaluate_manhattan_dp( x )
%EVALUATE_MANHATTAN_DP Summary of this function goes here
%   Detailed explanation goes here

executable_dir = '~/Code/indoor_context/build/';
executable_name = 'dp_optimization_wrapper';

sequences = {...
    'exeter_bursary'; ...
    'exeter_mcr1'; ...
    'lab_foyer1'; ...
    'lab_foyer2'; ...
    'lab_ground1'; ...
    'lab_kitchen1'; ...
    'magd_bedroom'; ...
    'magd_living'; ...
    'som_corr1'; ...
    };

sequences = {...
    'lab_foyer1'; ...
    'exeter_mcr1'; ...
    };

frame_stride = 50;

if (length(x) ~= 7)
    error(['Expected 7 parameters, got ' num2str(length(x))]);
end

corner_penalty = x(1);
occlusion_penalty = x(2);
mono_weight = x(3);
stereo_weight = x(4);
agreement_sigma = x(5);
agreement_weight = x(6);
occlusion_weight = x(7);

cmd = ['./' executable_name];
cmd = [cmd ' --mono_weight=' num2str(mono_weight)];
cmd = [cmd ' --stereo_weight=' num2str(stereo_weight)];
cmd = [cmd ' --3d_agreement_sigma=' num2str(agreement_sigma)];
cmd = [cmd ' --3d_agreement_weight=' num2str(agreement_weight)];
cmd = [cmd ' --3d_occlusion_weight=' num2str(occlusion_weight)];
cmd = [cmd ' --corner_penalty=' num2str(corner_penalty)];
cmd = [cmd ' --occlusion_penalty=' num2str(occlusion_penalty)];

% Fixed parameters
cmd = [cmd ' --frame_stride=' num2str(frame_stride)];
for i = 1:length(sequences)
    cmd = [cmd ' --sequence=' sequences{i}];
end

cd(executable_dir);
[status,output]=system(cmd);
if (status ~= 0)
    error(['Error in executable: ' output]);
end

lines = strsplit(output, char(10));  % char(10) is newline, '\n' does _not_ work

% the last line is always empty, we want the second-last line
y = str2num(lines{length(lines)-1});

end

