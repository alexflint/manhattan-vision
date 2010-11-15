function psi=get_psi(features, soln)

if nargin==0
  test;
  psi = [];
  return;
end

[ny nx nf] = size(features);
flat_ftrs = reshape(features, nx*ny, nf);
psi = [];
for label = 0:2
  mask = (soln.orients == label);
  psi = [ psi; flat_ftrs' * mask(:) ];
end
psi = [ psi; soln.num_walls; soln.num_occlusions ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
function test

% create mock inputs
features(:,:,1) = [ 1 2 3 4; 10 9 8 7 ];
features(:,:,2) = [ 1 0 1 1;  0 0 0 5 ];
orients = [ 0 0 0 1; 1 1 1 0 ];
soln = create_solution(orients, 6, 2);

% check outputs
psi = get_psi(features, soln);
assert(all(psi == [ 13 7 31 1 0 0 6 2 ]'));

% check equivalence with get_score
weights = [ 1 0   .5 1.6   -1 -2    6 2 ];
obj = create_objective(features, weights);
assert(weights*psi == get_score(obj, soln));
