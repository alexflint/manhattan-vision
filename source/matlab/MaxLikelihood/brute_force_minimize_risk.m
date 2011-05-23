function [ path risk paths logps ] = brute_force_minimize_risk( payoffs, loss )
%BRUTE_FORCE_MINIMIZE_RISK Minimizes risk of Manhattan-like paths by
%enumerating all possible paths and computing expected loss explicitly.
%   Only useful for validation of the efficient DP algorithm.
%   Complexity is O(H * 3^(W*2))

if (nargin < 2)
    loss = @default_loss;
end

risk = Inf;
[ paths logps ] = enumerate_paths([], 0, payoffs);
for i = 1:size(paths,1)
    x = paths(i, :);
    r = compute_risk(x, paths, logps, loss);
    if (r < risk)
        risk = r;
        path = x;
    end
end

end

% Recursive enumeration
function [ paths logps ] = enumerate_paths(path, logp, payoffs)
[ny nx] = size(payoffs);

if (length(path) == nx)
    paths = path;
    logps = logp;
else
    if (isempty(path))
        ya = 1;
        yb = ny;
    else
        yprev = path(length(path));
        ya = max(yprev-1, 1);
        yb = min(yprev+1, ny);
    end
    paths = [];
    logps = [];
    logsum = log_sum_exp(payoffs(ya:yb, length(path)+1));
    for y = ya:yb
        lp = logp + payoffs(y, length(path)+1) - logsum;
        [ vpath, vlogp ] = enumerate_paths([path y], lp, payoffs);
        paths = [paths; vpath];
        logps = [logps; vlogp];
    end
end
end


% Compute risk
function risk = compute_risk(x, paths, logps, loss)
risk = 0;
for i = 1:size(paths, 1)
    L = 0;
    for j = 1:size(paths,2)
        L = L + loss(j, x(j), paths(i,j));
    end
    P = exp(logps(i));
	risk = risk + P*L;
end
end


% Compute column-wise loss
function loss = default_loss(x, y_est, y_true)
loss = abs(y_est - y_true);
end
