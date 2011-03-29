clear

% get the bounds
[ x0 lower upper] = manhattan_optimization_bounds();

% precompute payoffs
evaluate_manhattan_dp(x0, 1);

% set up the problem
Problem.f = @(x) evaluate_manhattan_dp([1 exp(x')]);
x0 = log(x0(2:length(x0)));
lower = log(lower(2:length(lower)));
upper = log(upper(2:length(upper)));
opts.maxevals = 1000;
opts.showits = true;

% begin optimization
[y_min, x_min] = Direct(Problem, [lower; upper], opts);
x_min = exp(x_min');
