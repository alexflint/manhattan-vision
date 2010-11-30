function [soln,obj]=reconstruct(casedata, weights)

if nargin<2
  error(['No weight vector given to reconstruct()']);
end

%check weights.wall_penalty >= 0;
%check weights.occlusion_penalty >= 0;

obj = make_objective(casedata.pixel_features, casedata.wall_features, weights);
soln = dp_solve(casedata.frame, obj);
