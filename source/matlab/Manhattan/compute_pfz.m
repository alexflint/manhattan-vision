function [ p logp ] = compute_pfz( features, verts, weights )
%COMPUTE_PFZ Summary of this function goes here
%   Detailed explanation goes here

[ny nx nf] = size(features);
check nf == size(weights);

% Get the path
[path mask] = get_path(verts);

% Sum logarithms over the path
logp = 0;
for x = 1:nx
    y = round(path(x));
    fx = features(y,x,:);
    logp = logp + log(fx * weights);
end

% We're done
p = exp(logp);

end

