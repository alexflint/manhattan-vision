function [ y ] = log_sum_exp( x )
%LOG_SUM_EXP Compute log(sum(exp(v))) in a numerically stable fashion
%   Detailed explanation goes here

m = max(x);
y = log(sum(exp(x-m))) + m;

end

