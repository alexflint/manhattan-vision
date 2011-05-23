% run unit tests
get_psi;
get_pixel_loss;
get_score;
make_objective;
make_contra_objective;
pack_weights;
unpack_weights;

% load some real test data
cases = dp_load_cases('lab_kitchen1', 56);
c = cases(1);
[ny nx nf] = size(c.pixel_features);
[gny gnx ns] = size(c.wall_features);
grid_size = [gny gnx];  % use this order so that 'zeros(grid_size)' works

% setup some mock weights
weights = make_weights(rand(3,nf), rand(2,ns), 2, 3);

% make objective functions
obj = make_objective(c.pixel_features, c.wall_features, weights);
contra_obj = make_contra_objective( ...
    c.pixel_features, ...
    c.wall_features, ...
    weights, ...
    c.ground_truth);

% test some mock solutions
rand('state', 0);  % make the RNG deterministic
test_solns = cell(0);
test_solns{1} = c.ground_truth;
test_solns{2} = make_solution(zeros(ny, nx), zeros(grid_size), 1, 2);
test_solns{3} = make_solution(ones(ny, nx), ones(grid_size), 3, 1.5);
test_solns{4} = make_solution(ones(ny, nx)*2, -ones(grid_size), 0.1, 0.2);
test_solns{5} = make_solution(floor(rand(ny, nx)*3), floor(rand(grid_size)*3)-1, 6, 6);

% check agreement between get_score, get_psi, and get_pixel_loss
for i = 1:length(test_solns)
  %fprintf('********\n%d\n********\n\n', i);
  soln = test_solns{i};
  score = get_score(obj, soln);
  contra_score = get_score(contra_obj, soln);
  psi = get_psi(c.pixel_features, c.wall_features, soln);
  innerprod = pack_weights(weights) * psi;
  loss = get_pixel_loss(soln.orients, c.ground_truth.orients);
  
  check abs(innerprod/score - 1.0) < 1e-8;
  check abs((innerprod+loss)/contra_score - 1.0) < 1e-8;
end

wlen = 3*nf + 2*ns + 2;

% check dp_solve for negative wall penalties
wvec = zeros(1, 3*nf+2*ns+2);
wvec(length(wvec)-1) = -1;
reconstruct(cases(1), unpack_weights(wvec, nf, ns));


% check agreement between get_score and the score computed by
% dp_solve
test_weights = cell(0);
for i=1:wlen
  test_weights{length(test_weights)+1} = ...
      unpack_weights([1:wlen]==i, nf, ns);
end
test_weights{length(test_weights)+1} = make_weights(rand(3,nf), zeros(2,ns), 0, 0);
test_weights{length(test_weights)+1} = make_weights(zeros(3,nf), rand(2,ns), 0, 0);
test_weights{length(test_weights)+1} = make_weights(zeros(3,nf), zeros(2,ns), 1, 0);
test_weights{length(test_weights)+1} = make_weights(zeros(3,nf), zeros(2,ns), 0, 1);
test_weights{length(test_weights)+1} = make_weights(rand(3,nf), rand(2,ns), 2, 3);

for i = 95:length(test_weights)
  [soln obj] = reconstruct(cases(1), test_weights{i});
  score = get_score(obj, soln);
  if abs(soln.score) < 1e-8
    check abs(score) < 1e-8;
  else
    check abs(score/soln.score - 1.0) < 1e-6;
  end
end

% Just run this one to check that it works...
evaluate(cases, weights);

disp('All tests passed');
