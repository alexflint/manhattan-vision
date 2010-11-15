function obj=create_contra_objective(features, weights, gt_soln)

if nargin==0
  test1;
  test2;
  return;
end

% create the "regular obj"
obj = create_objective(features, weights);

% add one to all the wrong answers
% TODO: loss function per pixel should be an input parameter
for label = 0:2
  obj.scores(:,:,label+1) = obj.scores(:,:,label+1) + (gt_soln.orients ~= label);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test 1
% check the score computation within create_contra_obj
function test1

% setup mock features
features(:,:,1) = [ 1 2 3; 4 5 6; 7 8 9 ];
features(:,:,2) = [ 1 0 1; 0 1 0; 1 0 1 ];
features(:,:,3) = [ 2 2 2; 3 3 3; 4 4 4 ];

% setup mock weights
weights = [ 1 0 0   2 2 2    1 0 -1   -3 -2 ];  % last two are penalties

% setup mock ground truth
gt_orients = [ 0 0 0; 1 1 1; 2 2 2 ];
gt_soln = create_solution(gt_orients, 5, 4);

% test the return values
obj = create_contra_objective(features, weights, gt_soln);
assert(obj.wall_penalty == 3);
assert(obj.occlusion_penalty == 2);
assert(all(all(obj.scores(:,:,1) == [ 1  2  3;  5  6  7;  8  9 10 ])));
assert(all(all(obj.scores(:,:,2) == [ 9  9 13; 14 18 18; 25 25 29 ])));
assert(all(all(obj.scores(:,:,3) == [ 0  1  2;  2  3  4;  3  4  5 ])));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
% check that <w,psi_contra(x,y)> = <w,psi(x,y)> + loss(y,y_i)
function test2

% setup mock features
features(:,:,1) = [ 1 2 3; 4 5 6; 7 8 9 ];
features(:,:,2) = [ 1 0 1; 0 1 0; 1 0 1 ];
features(:,:,3) = [ 2 2 2; 3 3 3; 4 4 4 ];

% setup mock weights
weights = [ 1 0 0   2 2 2    1 0 -1    -2 -3 ];  % last two are penalties

% setup mock ground truth
gt_orients = [ 0 0 0; 1 1 1; 2 2 2 ];
gt_soln = create_solution(gt_orients, 6, 6);

% get the obj
obj = create_objective(features, weights);
contra_obj = create_contra_objective(features, weights, gt_soln);

% test size
assert(all(size(obj.scores) == [3 3 3]));

% setup other orients
test_orients = cell(3);
test_orients{1} = gt_orients;
test_orients{2} = [ 0 1 2; 2 1 0; 1 0 1 ];
test_orients{3} = ones(3,3);

for i = 1:length(test_orients)
  soln = create_solution(test_orients{i}, 5, 2.2);
  score = get_score(contra_obj, soln);
  psi = get_psi(features, soln);
  innerprod = weights * psi;
  loss = get_loss(test_orients{i}, gt_orients);
  assert(score == innerprod+loss);
end
