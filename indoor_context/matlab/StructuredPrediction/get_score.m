function score=get_score(obj, soln)
% Get the value of the function that the DP optimizes (ignoring
% regularlization terms) for a specific hypothesized set of
% orientations. This function is related to get_psi and
% make_objective as follows:
%   for all w, ftrs, and orients:
%     w'*get_psi(ftrs, orients) = 
%         get_score(make_objective(ftrs, w), orients)

if nargin==0
  test;
  return;
end

check size(obj.pixel_scores) == [size(soln.orients) 3];

score = 0;
for label=0:2
  slice = obj.pixel_scores(:,:,label+1);
  score = score + sum(sum(slice(soln.orients==label)));
end
%pixel_score = score;
%report pixel_score;

for i = 1:2
  slice = obj.wall_scores(:,:,i);
  score = score + sum(sum(slice(soln.path==(i-1))));
end
%wall_score = score - pixel_score;
%report wall_score;

score = score - soln.num_walls * obj.wall_penalty;
score = score - soln.num_occlusions * obj.occlusion_penalty;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
function test

% make mock inputs
pix_scores(:,:,1) = [ 1 2; -1 -1; -2 -1 ];
pix_scores(:,:,2) = [ 0 0;  1  0; -1  2 ];
pix_scores(:,:,3) = [ 0.5 0.5; 0.5 0.5; 0 -5 ];

wall_scores(:,:,1) = [ 10 20 30 40 ];
wall_scores(:,:,2) = [ 50 60 70 80 ];

obj = struct('pixel_scores', pix_scores, ...
             'wall_scores', wall_scores, ...
             'wall_penalty', 1, ...
             'occlusion_penalty', 3);

soln = make_solution([ 2 0; 0 1; 1 1 ], [0 0 1 -1], 1, 0);
check get_score(obj, soln) == 101.5;

soln = make_solution([ 0 0; 0 0; 0 0 ], [-1 -1 -1 -1], 4, 1);
check get_score(obj, soln) == -9;

soln = make_solution([ 2 2; 1 1; 0 0 ], [1 0 1 0], 3, 3);
check get_score(obj, soln) == 167;
