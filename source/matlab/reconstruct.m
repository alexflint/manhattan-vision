function [soln,obj]=reconstruct(casedata, weights)

if nargin<2
  [ny nx nf] = size(casedata.pixel_features);
  if nf==3
    weights = [ 0 0 1   0 1 0   0 0 1  -2 -1 ]
  elseif nf==6
    weights = [ 0 0 0 1 0 0    0 0 0 0 1 0    0 0 0 0 0 0 1   -2 -1 ];
  else
    error(['No weight vector given and unrecognised feature length: ' num2str(nf)]);
  end
end

obj = create_objective(casedata.pixel_features, weights);
soln = dp_solve(casedata.frame, obj);
