function score=get_score(obj, soln)
% Get the value of the function that the DP optimizes (ignoring
% regularlization terms) for a specific hypothesized set of
% orientations. This function is related to get_psi and
% create_objective as follows:
%   for all w, ftrs, and orients:
%     w'*get_psi(ftrs, orients) = 
%         get_score(create_objective(ftrs, w), orients)

if nargin==0
  test;
  score = [];
  return;
end

score = 0;
for label=0:2
  s = obj.scores(:,:,label+1);
  score = score + sum(sum(s(soln.orients==label)));
end
score = score - soln.num_walls * obj.wall_penalty;
score = score - soln.num_occlusions * obj.occlusion_penalty;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit test
function test

% create mock inputs
scores(:,:,1) = [ 1 2; -1 -1; -2 -1 ];
scores(:,:,2) = [ 0 0;  1  0; -1  2 ];
scores(:,:,3) = [ 0.5 0.5; 0.5 0.5; 0 -5 ];
obj = struct('scores', scores, ...
             'wall_penalty', 1, ...
             'occlusion_penalty', 3);

soln = create_solution([ 2 0; 0 1; 1 1 ], 1, 0);
assert(get_score(obj, soln) == 1.5);

soln = create_solution([ 0 0; 0 0; 0 0 ], 4, 1);
assert(get_score(obj, soln) == -9);

soln = create_solution([ 2 2; 1 1; 0 0 ], 3, 3);
assert(get_score(obj, soln) == -13);
