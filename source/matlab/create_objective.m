function objective=create_objective(features, weights)

if nargin==0
  test;
  objective=[];
  return;
end

if nargin<2
  if size(features,3)==6
    weights = [ 0 0 0 1 0 0    0 0 0 0 1 0    0 0 0 0 0 0 1 ];
  elseif size(features,3)==3
    weights = [ 1 0 0   0 1 0   0 0 1 ];
  end
end

if size(weights,2) ~= 1
  weights = weights';
end

% flatten the features
[ny nx nf] = size(features);
assert(all(size(weights) == [nf*3+2, 1]));

% compute scores for each orientation
flat_ftrs = reshape(features, nx*ny, nf);
scores = zeros(ny, nx, 3);
for label = 0:2
  classifier = weights(label*nf+1 : (label+1)*nf);
  scores(:,:,label+1) = reshape(flat_ftrs*classifier, ny, nx);
end

objective = struct(...
    'scores', scores, ...
    'wall_penalty', -weights(length(weights)-1), ...  % note minus
    'occlusion_penalty', -weights(length(weights)) ... % note minus
    );




% Unit test
function test

% setup mock features
features(:,:,1) = [ 1 2 3; 4 5 6; 7 8 9 ];
features(:,:,2) = [ 1 0 1; 0 1 0; 1 0 1 ];
features(:,:,3) = [ 2 2 2; 3 3 3; 4 4 4 ];

weights = [ 1 0 0   2 2 2    1 0 -1    -2 -7.5 ];

obj = create_objective(features, weights);
assert(obj.wall_penalty == 2);
assert(obj.occlusion_penalty == 7.5);
assert(all(all(obj.scores(:,:,1) == features(:,:,1))));
assert(all(all(obj.scores(:,:,2) == 2*(features(:,:,1)+features(:,:,2)+features(:,:,3)))));
assert(all(all(obj.scores(:,:,3) == features(:,:,1) - features(:,:,3))));
