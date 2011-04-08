function [ y output ] = evaluate_manhattan_dp( x, precompute, args )
%EVALUATE_MANHATTAN_DP Summary of this function goes here
%   Detailed explanation goes here

if (nargin < 2)
    precompute = 0;
end

if (nargin < 3)
    args = '';
end
    
    
if (length(x) ~= 6)
    error(['Expected 6 parameters, got ' num2str(length(x))]);
end

executable_dir = '~/Code/indoor_context/build/';
executable_name = 'dp_optimization_wrapper';

%sequences = {...
%    'exeter_bursary'; ...
%    'exeter_mcr1'; ...
%    'lab_foyer1'; ...
%    'lab_foyer2'; ...
%    'lab_ground1'; ...
%    'lab_kitchen1'; ...
%    'magd_bedroom'; ...
%    'magd_living'; ...
%    'som_corr1'; ...
%    };

sequences = {...
    'lab_foyer1'; ...
    'lab_foyer2'; ...
    'som_corr1'; ...
    'lab_ground1'; ...
    'lab_kitchen1'; ...
    };

frame_stride = 10;

% Get parameters from the vector
corner_penalty = x(1);
occlusion_penalty = x(2);
mono_weight = x(3);
stereo_weight = x(4);
agreement_weight = x(5);
occlusion_weight = x(6);

% Build the command
cmd = ['./' executable_name];
cmd = [cmd ' --corner_penalty=' num2str(corner_penalty)];
cmd = [cmd ' --occlusion_penalty=' num2str(occlusion_penalty)];
cmd = [cmd ' --mono_weight=' num2str(mono_weight)];
cmd = [cmd ' --stereo_weight=' num2str(stereo_weight)];
cmd = [cmd ' --3d_agreement_weight=' num2str(agreement_weight)];
cmd = [cmd ' --3d_occlusion_weight=' num2str(occlusion_weight)];
cmd = [cmd ' --frame_stride=' num2str(frame_stride)];
for i = 1:length(sequences)
    cmd = [cmd ' --sequence=' sequences{i}];
end

if (precompute)
    cmd = [cmd ' --store_payoffs=PrecomputedData'];
else
    cmd = [cmd ' --load_payoffs=PrecomputedData'];
end

cmd = [cmd ' ' args];

% Print command
fprintf('Command: %s\n', cmd);

% Execute command
cd(executable_dir);
[status,output]=system(cmd);
if (status ~= 0)
    error(['Error in executable: ' output]);
end

if (precompute)
    disp('Precomputed payoffs');
    y = nan;
else
    lines = strsplit(output, char(10));  % char(10) is newline, '\n' fails
    y = str2double(lines{length(lines)-1});
    fprintf('  Result: %f\n', y);
end

end
