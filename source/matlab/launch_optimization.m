
[ x0 lower_bound upper_bound ] = manhattan_optimization_bounds();
   
% precompute payoff features
evaluate_manhattan_dp(x0, 1);

% start optimizing
f = @(x) evaluate_manhattan_dp(exp(x));
[ymin xmin] = gpgo(f, log(x0), log(lower_bound), log(upper_bound));
xmin = exp(xmin);
