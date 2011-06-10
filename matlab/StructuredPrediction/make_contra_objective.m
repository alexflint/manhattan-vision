function obj=make_contra_objective(pixel_features, wall_features, weights, gt_soln)

if nargin==0
  test1;
  test2;
  return;
end

if isstruct(gt_soln)
  gt_orients = gt_soln.orients;
else
  gt_orients = gt_soln;
end

check size(gt_orients) == [size(pixel_features,1) size(pixel_features,2)];

obj = make_objective(pixel_features, wall_features, weights);

% add one to all the wrong answers
% TODO: loss function per pixel should be an input parameter
for i = 1:3
  obj.pixel_scores(:,:,i) = obj.pixel_scores(:,:,i) + (gt_orients ~= i-1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test 1
% check the score computation within make_contra_obj
function test1

% setup mock features
features(:,:,1) = [ 1 2 3; 4 5 6; 7 8 9 ];
features(:,:,2) = [ 1 0 1; 0 1 0; 1 0 1 ];
features(:,:,3) = [ 2 2 2; 3 3 3; 4 4 4 ];
wall_features(:,:,1) = [ -10 -20 -30 -40 ; 50 60 70 80 ];
wall_features(:,:,2) = [  0   2   -1  3  ; 7  13 -3 11 ];

% setup mock weights
weights = make_weights([1 0 0; 2 2 2; 1 0 -1], [ 1 -1; 10 10], 3, 2);

% setup mock ground truth
gt_orients = [ 0 0 0; 1 1 1; 2 2 2 ];
gt_path = [ 0 -1 1 -1 ; -1 1 1 0 ];
gt_soln = make_solution(gt_orients, gt_path, 5, 4);

% test the return values
obj = make_contra_objective(features, wall_features, weights, gt_soln);

expected_pix_scores(:,:,1) = [ 1  2  3;  5  6  7;  8  9 10 ];
expected_pix_scores(:,:,2) = [ 9  9 13; 14 18 18; 25 25 29 ];
expected_pix_scores(:,:,3) = [ 0  1  2;  2  3  4;  3  4  5 ];

expected_wall_scores(:,:,1) = [ -10 -22 -29 -43 ; 43 47 73 69 ];
expected_wall_scores(:,:,2) = [ -100 -180 -310 -370 ; 570 730 670 910 ];

check obj.pixel_scores == expected_pix_scores;
check obj.wall_scores == expected_wall_scores;
check obj.wall_penalty == 3;
check obj.occlusion_penalty == 2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
% check that <w,psi_contra(x,y)> = <w,psi(x,y)> + loss(y,y_i)
function test2

% setup mock features
features(:,:,1) = [ 1 2; 4 5; 7 8; 13 17 ];
features(:,:,2) = [ 1 0; 0 1; 1 0; 21 12 ];
wall_features = [ 10 -20 ; -30 40 ];

% setup mock weights
weights = make_weights([1 0; 2 2; 1 0], [3; -5], 1, 4);

% setup mock ground truth
gt_orients = [ 0 1; 2 0; 1 2; 0 1 ];
gt_path = [ 1 -1; 0 1 ];
gt_soln = make_solution(gt_orients, gt_path, 6, 6);

% get the obj
obj = make_objective(features, wall_features, weights);
contra_obj = make_contra_objective(features, wall_features, weights, gt_soln);

% test size
check size(obj.pixel_scores) == [4 2 3];

% setup other orients
test_orients{1} = gt_orients;
test_orients{2} = [ 0 1; 2 1; 1 0; 1 2 ];
test_orients{3} = ones(4,2);

test_paths{1} = gt_path;
test_paths{2} = [ -1 -1 ; -1 -1 ];
test_paths{3} = [ -1 0 ; 1 -1 ];

% check consistency for each test_orients
for i = 1:length(test_orients)
  soln = make_solution(test_orients{i}, test_paths{i}, 5, 2.2);
  score = get_score(contra_obj, soln);
  psi = get_psi(features, wall_features, soln);
  innerprod = pack_weights(weights) * psi;
  loss = get_pixel_loss(test_orients{i}, gt_orients);
  check score == innerprod+loss;
end
