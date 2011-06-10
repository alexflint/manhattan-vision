function y=summ(x)
% sum over all dimensions
y = x;
while prod(size(y)) > 1
  y=sum(y);
end
