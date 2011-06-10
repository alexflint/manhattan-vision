function weights = unpack_weights(wvec, num_pix_ftrs, num_wall_ftrs)

if nargin==0
  test;
  return;
end

check nargin == 3;

nf = num_pix_ftrs;
ns = num_wall_ftrs;

check size(wvec) == [1, nf*3 + ns*2 + 2];

wpix = reshape(wvec(1:nf*3), nf, 3)';
wwall = reshape(wvec(nf*3+1 : nf*3+ns*2), ns, 2)';
wall_penalty = -wvec(length(wvec)-1);
occl_penalty = -wvec(length(wvec));

weights = make_weights(wpix, wwall, wall_penalty, occl_penalty);





% Unit test
function test

wvec = [ 1 2   3 4    5 6     7 8 9     10 11 12    -13 -14 ];
weights = unpack_weights(wvec, 2, 3);

expected_pw = [ 1 2 ; 3 4 ; 5 6 ];
expected_ww = [ 7 8 9 ; 10 11 12 ];

check weights.pixel_weights == expected_pw;
check weights.wall_weights == expected_ww;
check weights.wall_penalty == 13;
check weights.occlusion_penalty == 14;
