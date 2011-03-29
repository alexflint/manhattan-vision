function brief_disp(expr, val)
% Report a value but avoid producing copious output by replacing the
% output for large matrices by their dimensions.

disp([expr ' = [' describe_var(val) ']']);

dims = size(val);
if prod(dims) <= 100
  disp(val);
end
