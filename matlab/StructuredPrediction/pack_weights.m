function wvec = pack_weights(weights)

if nargin==0
  test;
  return;
end

% careful with transposing here, it changes the order below
pt = weights.pixel_weights';
wt = weights.wall_weights';

wvec = [ pt(:)', ...
         wt(:)', ...
         -weights.wall_penalty, ...  % note minus sign
         -weights.occlusion_penalty ... % note minus sign
       ];






% Unit test
function test

w = make_weights([ 1 2 3 4 ; 5 6 7 8 ; 9 10 11 12 ], ...
                 [ -1 -2 -3 -4 ; -5 -6 -7 -8 ], ...
                 20, 30);

expected_wvec = [ 1 2 3 4 5 6 7 8 9 10 11 12 ...
                 -1 -2 -3 -4 -5 -6 -7 -8 ...
                 -20 -30 ];

wvec = pack_weights(w);
assert(all(wvec == expected_wvec));