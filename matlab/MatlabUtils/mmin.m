function y=mmin(x)
% take the min over all dimensions
y = x;
while prod(size(y)) > 1
  y=min(y);
end
