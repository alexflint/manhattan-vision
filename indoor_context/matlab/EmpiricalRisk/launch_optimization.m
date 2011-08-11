
[ x0 lower upper ] = manhattan_optimization_bounds();
   
% precompute payoff features
evaluate_manhattan_dp(x0, 1);

% start optimizing
f = @(x) evaluate_manhattan_dp([1 exp(x)]);
x0 = log(x0(2:length(x0)));
lower = log(lower(2:length(lower)));
upper = log(upper(2:length(upper)));
[ymin xmin] = gpgo(f, x0, lower, upper);
xmin = exp(xmin);
