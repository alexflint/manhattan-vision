% load some real test data
cases = dp_load_cases('lab_kitchen1', 56);
c = cases(1);
[ny nx nf] = size(c.pixel_features);
[gny gnx ns] = size(c.wall_features);

% create objective function
wvec = [rand(1, nf*3 + ns*2) -1 -1];
weights = unpack_weights(wvec, nf, ns);
obj = make_objective(c.pixel_features, c.wall_features, weights);

% run the test
profile on
tic;
for i=1:20
  soln = dp_solve(c.frame, obj);
end
t=toc;
disp(['Average dp_solve execution time: ' num2str(t/20)]);
profile viewer;
