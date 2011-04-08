function [ loglik J_loglik cmd ] = compute_loglik( params, feature_set, args )
%COMPUTE_LOGLIK Summary of this function goes here
%   Detailed explanation goes here

% Check input arguments
if (nargin < 3)
    args = '';
    if (nargin < 2)
        feature_set = 'PrecomputedData/features.protodata';
    end
end

executable_dir = '~/Code/indoor_context/build/';
executable_name = 'compute_loglik';

% Get parameters from the vector
corner_penalty = params(1);
occlusion_penalty = params(2);
theta = params(3:length(params));

% Build the command. Important to use high precisions here so that no data
% is lost in transmission to c++.
cmd = ['./' executable_name];
cmd = [cmd ' --corner_penalty=' num2str(corner_penalty, '%.18f')];
cmd = [cmd ' --occlusion_penalty=' num2str(occlusion_penalty, '%.18f')];
cmd = [cmd ' --weights=''' num2str(theta, '%.18f') ''''];
cmd = [cmd ' --features=' feature_set];
cmd = [cmd ' ' args];

% Print command
fprintf('Command: %s\n', cmd);

% Execute command and parse results
[ loglik J_loglik ] = run_executable(cmd, executable_dir);

end
