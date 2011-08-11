function y=mmax(x)
% take the min over all dimensions
y = x;
while prod(size(y)) > 1
  y=max(y);
end
