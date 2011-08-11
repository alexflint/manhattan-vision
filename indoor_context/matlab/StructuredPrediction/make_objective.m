function objective=make_objective(pixel_features, wall_features, weights)

if nargin==0
  test;
  objective=[];
  return;
end

if nargin<3
  if size(pixel_features,3)==6
    weights = [ 0 0 0 1 0 0    0 0 0 0 1 0    0 0 0 0 0 0 1 ];
  elseif size(pixel_features,3)==3
    weights = [ 1 0 0   0 1 0   0 0 1 ];
  end
end

if ~isstruct(weights)
  error('weights should be a struct (use make_weights)');
end

% flatten the features
[ny nx nf] = size(pixel_features);
[gny gnx ns] = size(wall_features);
if (gny == 0) 
    ns = 0;
end

% compute pixel scores
flat_ftrs = reshape(pixel_features, nx*ny, nf);
pixel_scores = zeros(ny, nx, 3);
for i = 1:3
  classifier = weights.pixel_weights(i,:);
  pixel_scores(:,:,i) = reshape(flat_ftrs*classifier', ny, nx);
end

% compute wall scores
wall_scores = zeros(gny, gnx, 2);
for i = 1:2
  for s = 1:ns
    slice = wall_features(:,:,s);
    k = weights.wall_weights(i,s);
    wall_scores(:,:,i) = wall_scores(:,:,i) + slice*k;
  end
end

% consruct the final objective
objective = struct(...
    'pixel_scores', pixel_scores, ...
    'wall_scores', wall_scores, ...
    'wall_penalty', weights.wall_penalty, ...
    'occlusion_penalty', weights.occlusion_penalty ...
    );






% Unit test
function test

% setup mock features
pixftrs(:,:,1) = [ 1 2 3; 4 5 6; 7 8 9 ];
pixftrs(:,:,2) = [ 1 0 1; 0 1 0; 1 0 1 ];
pixftrs(:,:,3) = [ 2 2 2; 3 3 3; 4 4 4 ];

wallftrs(:,:,1) = [ 1 2 ; 3 4 ];
wallftrs(:,:,2) = [ 0 1 ; 0 1 ];

weights = make_weights([1 0 0; 2 2 2; 1 0 -1],   [1 -2; 0 -1],  2, 7.5);

obj = make_objective(pixftrs, wallftrs, weights);

check obj.wall_penalty == 2;
check obj.occlusion_penalty == 7.5;
check obj.pixel_scores(:,:,1) == pixftrs(:,:,1);
check obj.pixel_scores(:,:,2) == 2*(pixftrs(:,:,1)+pixftrs(:,:,2)+pixftrs(:,:,3));
check obj.pixel_scores(:,:,3) == pixftrs(:,:,1) - pixftrs(:,:,3);
check obj.wall_scores(:,:,1) == wallftrs(:,:,1)-2*wallftrs(:,:,2);
check obj.wall_scores(:,:,2) == -wallftrs(:,:,2);
