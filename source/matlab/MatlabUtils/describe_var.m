function s=describe_var(x)
% Get a string describing the type and dimensions of a variable

dims = size(x);
dimstr = num2str(dims(1));
for i = 2:length(dims)
  dimstr = [ dimstr 'x' num2str(dims(i)) ];
end
s=[dimstr ' ' class(x)];
