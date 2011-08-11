function [ err ] = test_loglik_jacs(h)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

default h = 1e-5;

% arbitrary (but repeatable) choice
x0 = [ 4 2.5 1.5 .1 210.3 10 ];

% compare with finite differences
err = verify_jacobians( @(x)compute_loglik(x), x0, h );
