function [ v ] = test_loglik_jacs( x0 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

[ model_loglik J_model_loglik ftr_loglik J_ftr_loglik ] = compute_loglik(x0);
lambda0 = x0(1:2);
wts0 = x0(3:6);

report model_loglik;
report ftr_loglik;

h = 1e-3;

partial_model_f = @(lambda)(model_loglik_f(lambda, wts0));
num_J_model_loglik = numeric_derivative(partial_model_f, lambda0, h);

partial_ftr_f = @(wts)(ftr_loglik_f(lambda0, wts));
num_J_ftr_loglik = numeric_derivative(partial_ftr_f, wts0, h);

report J_model_loglik;
report num_J_model_loglik;
disp(['Error: ' num2str(norm(J_model_loglik - num_J_model_loglik))]);

report J_ftr_loglik;
report num_J_ftr_loglik;
disp(['Error: ' num2str(norm(J_ftr_loglik - num_J_ftr_loglik))]);

end

function [model_loglik J_model_loglik] = model_loglik_f(lambda, wts)
[ model_loglik J_model_loglik ftr_loglik J_ftr_loglik ] = compute_loglik([lambda wts]);
end

function [ftr_loglik J_ftr_loglik] = ftr_loglik_f(lambda, wts)
[ model_loglik J_model_loglik ftr_loglik J_ftr_loglik ] = compute_loglik([lambda wts]);
end
