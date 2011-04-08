function [ logpost post ] = compute_posteriors( payoffs )
%COMPUTE_POSTERIORS Compute posteriors on a simplified Manhattan-like grid
%   Detailed explanation goes here

[ny nx] = size(payoffs);
logpost = zeros(ny, nx);

logpost(:,1) = payoffs(:,1) - log_sum_exp(payoffs(:,1));

for x = 1:nx-1
    next = zeros(ny,1);
    for y = 1:ny
        za = max(1, y-1);
        zb = min(ny, y+1);
        
        norm = log_sum_exp(payoffs(za:zb,x+1));
        for z = za:zb
            next(z) = next(z) + exp(logpost(y,x) + payoffs(z,x+1) - norm);
        end
    end
    
    logpost(:,x+1) = log(next);
    
    %report x;
    %report sum(exp(logpost(:,x)));    
end

post = exp(logpost);