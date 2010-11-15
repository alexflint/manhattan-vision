% run unit tests
get_psi;
get_loss;
get_score;
create_objective;
create_contra_objective;

% load some real test data
cases = dp_load_cases('lab_kitchen1', 56);
c = cases(1);
[ny nx nf] = size(c.pixel_features);

% setup some mock weights
weights = rand(nf*3+2, 1);
weights(nf*3+1) = -weights(nf*3+1);  % these must be negative
weights(nf*3+2) = -weights(nf*3+2);  % these must be negative 

% create objective functions
obj = create_objective(c.pixel_features, weights);
contra_obj = create_contra_objective(c.pixel_features, weights, ...
                                     c.ground_truth);

% test some mock solutions
rand('state', 0);  % make the random number deterministic
test_solns = cell(0);
test_solns{1} = c.ground_truth;
test_solns{2} = create_solution(zeros(ny, nx), 1, 2);
test_solns{3} = create_solution(ones(ny, nx), 3, 1.5);
test_solns{4} = create_solution(ones(ny, nx)*2, 0.1, 0.2);
test_solns{5} = create_solution(floor(rand(ny, nx)*3), 6, 6);

for i = 1:length(test_solns)
  soln = test_solns{i};
  score = get_score(obj, soln);
  contra_score = get_score(contra_obj, soln);
  psi = get_psi(c.pixel_features, soln);
  innerprod = weights' * psi;
  loss = get_loss(soln.orients, c.ground_truth.orients);
  assert(abs((score-innerprod)/score) < 1e-8);
  assert(abs((contra_score - (innerprod+loss))/contra_score) < 1e-8);
end

% Just run this one to check that it works...
evaluate(cases, weights);

disp('All tests passed');
