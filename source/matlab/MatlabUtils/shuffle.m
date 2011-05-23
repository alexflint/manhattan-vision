function ys = shuffle(xs)
% Shuffle an array

ys = xs(randperm(length(xs)));