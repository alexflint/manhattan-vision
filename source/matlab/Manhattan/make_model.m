function [ model ] = make_model( xs, ys, as )
%MAKE_MODEL Summary of this function goes here
%   Detailed explanation goes here

check size(xs, 2) == 1;
check size(ys, 2) == 1;
check size(as, 2) == 1;

model = [ xs ys as ];

end

