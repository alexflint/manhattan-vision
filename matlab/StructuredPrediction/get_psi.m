function psi=get_psi(pixel_features, wall_features, soln)

if nargin==0
  test;
  return;
end

[ny nx nf] = size(pixel_features);
flat_ftrs = reshape(pixel_features, nx*ny, nf);

% compute psi for pixel-wise features
pixel_psi = [];
for label = 0:2
  mask = (soln.orients == label);
  pixel_psi = [ pixel_psi; flat_ftrs' * mask(:) ];
end

% compute psi for column-wise features
check [size(wall_features,1) size(wall_features,2)] == size(soln.path);
ns = size(wall_features,3);
wall_psi = zeros(ns*2, 1);
for i = 0:1
  for s = 1:ns
    slice = wall_features(:,:,s);
    wall_psi(i*ns+s) = sum(sum(slice(soln.path==i)));
  end
end

psi = [ pixel_psi; wall_psi; soln.num_walls; soln.num_occlusions ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
function test

% make mock inputs
pix_features(:,:,1) = [ 1 2 3 4; 10 9 8 7 ];
pix_features(:,:,2) = [ 1 0 1 1;  0 0 0 5 ];

wall_features(:,:,1) = [ 1 ; 10 ; 100 ; 1000 ];
wall_features(:,:,2) = [ 2 ; 20 ; 200 ; 2000 ];
wall_features(:,:,3) = [ 3 ; 30 ; 300 ; 3000 ];

orients = [ 0 0 0 1; 1 1 1 0 ];
path = [ 1 ; 0 ; -1 ; 1 ];
soln = make_solution(orients, path, 6, 2);

% check outputs
psi = get_psi(pix_features, wall_features, soln);
check psi == transpose([ 13 7 31 1 0 0    10 20 30  1001 2002 3003    6 2 ]);

% check equivalence with get_score
weights = make_weights([1 0; .5 1.6; -1 -2], [10 5 -2 ; 0 0.5 -6], 6, 2);
obj = make_objective(pix_features, wall_features, weights);
check pack_weights(weights)*psi == get_score(obj, soln);
