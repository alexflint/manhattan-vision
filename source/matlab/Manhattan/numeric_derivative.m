function [ grad ys ] = numeric_derivative(f, x0, h)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if nargin == 0
    test;
    disp('Test passed');
    return;
end

xdim = length(x0);
if (nargin < 3)
    h = ones(1, xdim);
end
if (isscalar(h))
    h = h*ones(1,xdim);
end

if (~strcmp(class(f), 'function_handle'))
    error(['f should be a function, but it was: ' class(f)]);
end

% Do evaluations
grad = zeros(size(x0));
y0 = f(x0);
ys = zeros(xdim, 3);
for i = 1:xdim
    step = h .* (1:xdim==i);
    xa = x0-step;
    xb = x0+step;
    ys(i,1) = f(xa);
    ys(i,2) = y0;
    ys(i,3) = f(xb);
    diffs = gradient(ys(i,:), h(i));
    grad(i) = diffs(2);
end

end



function test

f = @(x)(exp(x(1)) + 2*x(2) + x(3)^2);
g = numeric_derivative(f, [1 0 3], 1e-8);
err = abs(g - [exp(1) 2 6]);
check norm(err) < 1e-6

end