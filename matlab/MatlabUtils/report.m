function report(varargin)
% Report an expression and its value
% Usage:
%   report x == y
%   report size(x) > 0
%   report 2*x ~= foo(z)
% Note that CHECK _only_ works without the brackets. For example,
% check(f==g) will _not_ work expected.

% Concatenate all the parameters together to form an expression
% string and evaluate in the parent context.
str = [varargin{:}];

delims = [0 strfind(str, '$') length(str)+1];
for i = 1:length(delims)-1
  expr = str(delims(i)+1:delims(i+1)-1);
  try
    val = evalin('caller', expr);
  catch ex
    ex.throwAsCaller();
  end
  disp([expr ' =']);
  disp(val);
end
