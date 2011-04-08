function [ path risk paths posteriors ] = naive_minimize_risk( payoffs, loss )
%BRUTE_FORCE_MINIMIZE_RISK Minimizes risk of Manhattan-like paths by
% computing losses w.r.t. element-wise posteriors, which are computed
% seperately.

if (nargin < 2)
    loss = @default_loss;
end

[ny nx] = size(payoffs);

% Compute posteriors for each cell
[ logpost posteriors ] = compute_posteriors(payoffs);

% Enumerate all possible paths
paths = enumerate_paths([], 0, payoffs);

risk = Inf;
for i = 1:size(paths,1)
    p = paths(i, :);
    r = 0;
    for x = 1:nx
        for y = 1:ny
            r = r + loss(x, p(x), y) * posteriors(y, x);
        end
    end
    if (r < risk)
        risk = r;
        path = p;
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


% Compute column-wise loss
function loss = default_loss(x, y_est, y_true)
loss = abs(y_est - y_true);
end
