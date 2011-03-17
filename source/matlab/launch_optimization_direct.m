clear

[ x0 lower_bound upper_bound ] = manhattan_optimization_bounds();

% precompute payoffs
evaluate_manhattan_dp(x0, 1);

% begin optimization
Problem.f = @(x) evaluate_manhattan_dp(exp(x'));
opts.maxevals = 1000;
opts.showits = true;
bounds = [lower_bound; upper_bound]';
[y_min, x_min] = Direct(Problem, log(bounds), opts);
x_min = exp(x_min');
