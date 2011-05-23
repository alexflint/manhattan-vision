function [ loglik J_loglik ] = mock_compute_model_loglik( lambda )
%MOCK_COMPUTE_MODEL_LOGLIK Summary of this function goes here
%   Detailed explanation goes here

% Mock values from lab_kitchen1:0
num_walls = 5;
num_occlusions = 1;

n1 = num_walls;
n2 = num_occlusions;

e1 = exp(-lambda(1));
e2 = exp(-lambda(2));
e12 = exp(-lambda(1)-lambda(2));
loglik = -lambda(1)*n1 - lambda(2)*n2 - log(1-e1-e2+e12);

r = [ e1-e12 ; e2-e12 ];
J_loglik = -[n1;n2] - 1/(1-e1-e2+e12) * r;

end
