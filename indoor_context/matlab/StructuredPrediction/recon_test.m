% load some real test data
cases = dp_load_cases('lab_kitchen1', 56);
c = cases(1);
[ny nx nf] = size(c.pixel_features);
[gny gnx ns] = size(c.wall_features);

wvec = zeros(1, nf*3 + ns*2 + 2);

% setup some mock weights
%weights = make_weights(zeros(nf, 3), zeros(ns, 2), 0, -1);
weights = unpack_weights(wvec, nf, ns);

% create objective functions
[soln, obj] = reconstruct(c, weights);
imshow(orientim(soln.orients));