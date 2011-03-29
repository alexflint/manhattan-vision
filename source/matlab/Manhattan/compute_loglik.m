function [ model_loglik J_model_loglik ftr_loglik J_ftr_loglik cmd ] = compute_loglik( x, feature_set, args )
%COMPUTE_LOGLIK Summary of this function goes here
%   Detailed explanation goes here

if (length(x) ~= 6)
    error(['Expected 6 parameters, got ' num2str(length(x))]);
end

if (nargin < 3)
    args = '';
end

if (nargin < 2)
    feature_set = 'PrecomptuedData/features.protodata';
end

executable_dir = '~/Code/indoor_context/build/';
executable_name = 'compute_loglik';

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
cmd = [cmd ' --features=' feature_set];
cmd = [cmd ' ' args];

% Print command
fprintf('Command: %s\n', cmd);

% Execute command and parse results
[ model_loglik J_model_loglik ftr_loglik J_ftr_loglik ] = run_executable(cmd, executable_dir);

end
