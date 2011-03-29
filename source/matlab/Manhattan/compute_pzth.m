function [ p logp ] = compute_pzth( ncorners, noccl, lambda )
%COMPUTE_PZTH Summary of this function goes here
%   Detailed explanation goes here

check length(lambda) == 2;

logp = [ncorners noccl] .* lambda;
p = exp(logp);

end

