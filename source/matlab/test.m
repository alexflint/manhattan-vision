cases = dp_load_cases('lab_kitchen1', 5, 'sweeps_only');
c = cases(1);

w = [ 0 0 0 0 0 0 0 0 0      10 0.1   -2 -3 ];
[r obj] = reconstruct(c, w);

imshow(orientim(r,c));
