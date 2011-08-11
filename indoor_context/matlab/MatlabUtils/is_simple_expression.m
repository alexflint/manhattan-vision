function r=is_simple_expression(expr)
% This function guesses whether an expression string is simple in
% the sense that it contains no variables or function calls. This
% is used by check.m to avoid printing expressions with values that
% can be read directly from the expression. Examples of simple
% expressions are:
%   '2'
%   '(40+60)/(2^3)'
%   '[ 10 20 30 ]'

% Use a crude test that just checks whether the entire expression
% is formed from numbers and simple operators. The character 'e' is
% in here to catch expressions like '1e+8', but obviously just
% including 'e' here is an imperfect test.
simplechars = [ '0':'9' '[](){}+-*/^''.<>=~e ' ];

for i = 1:length(expr)
  if (isempty(strfind(simplechars, expr(i))))
    r = 0;
    return;
  end
end
r = 1;

