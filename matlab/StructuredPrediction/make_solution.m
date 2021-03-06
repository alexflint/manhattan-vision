function soln=make_solution(orients, path, num_walls, num_occlusions)
% Create a solution structure

soln = struct('orients', orients, ...
              'path', path, ...
              'num_walls', num_walls, ...
              'num_occlusions', num_occlusions ...
              );