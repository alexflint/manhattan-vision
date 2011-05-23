function check(varargin)
% Usage:
%   check x == y
%   check size(x) > 0
%   check 2*x ~= foo(z)
% Note that CHECK _only_ works without the brackets. For example,
% check(f==g) will not work as expected.

% Concatenate all the parameters together to form an expression string.
expr = [varargin{:}];

% First try to create the function without evaluating it. An error
% here indicates invalid *syntax* in the matlab statement passed to
% us, so we bail immediately as we cannot evaluate any variables in
% the expression.
try
  % pass nargin and nargout explicitly so that they are equal to
  % the value in the calling function rather than the value in this
  % anonymous function.
  f = evalin('caller', sprintf('@(nargin,nargout)%s', expr));
catch me
  me.throwAsCaller();
end

% Next try to evaluate the function we created. An error here
% indicates invalid *semantics* (e.g. mismatched matrix dimensions,
% undefined operators, etc) so we catch the exception and continue to
% report the relevant expressions, before rethrowing the exception at
% the end.
try
  % nargin and nargout do _not_ appear in CONTEXT, so we have to
  % get their values explicitly with evalin.
  caller_nargin = evalin('caller', 'nargin');
  caller_nargout = evalin('caller', 'nargout');
  % put this into an eval() so that we can catch an exception
  % whenever something goes wrong.
  pass = eval(sprintf('f(%d,%d)', caller_nargin, caller_nargout));
catch me
    % will rethrow later (after reporting as many variables as possible)
    ex = me;
    pass = 0;
end

% Collapse r to a scalar
while numel(pass) > 1
  pass = all(pass);
end

if ~pass
  finfo = functions(f);
  context = finfo.workspace{1};  % get info about the function
  vars = fieldnames(context);

  % print the relevant variables
  for i = 1:length(vars)
    var = vars{i};
    val = context.(var);
    brief_disp(var, val);
  end

  % pull apart the expression
  comps = {'==', '~=', '<', '<=', '>', '>='};
  for i = 1:length(comps)
    pos = strfind(expr, comps{i});
    if length(pos)==1
      lexpr = expr(1:pos-1);
      rexpr = expr(pos+length(comps{i}):length(expr));
      if ~is_simple_expression(lexpr) && ~isvarname(lexpr)   % avoid reporting trivial expressions
        lval = evalin('caller', lexpr);
        brief_disp(lexpr, lval);
      end
      if ~is_simple_expression(rexpr) && ~isvarname(rexpr)
        rvalue = evalin('caller', rexpr);
        brief_disp(rexpr, rvalue);
      end
      break;
    end
  end
  
  if exist('ex', 'var')
    ex.throwAsCaller();  % this hides check() from the stack frame
  else
    message = ['Assertion failed: ' expr];
    transparent_error(message);
  end
  % use dbstack here to omit this function from the stack trace
  % printed by matlab
end
end


function transparent_error(errstr)
% raise an error which does not include the check() function in the
% stack
error(struct('message', errstr, 'stack', dbstack(2)));
end