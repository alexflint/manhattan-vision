function [ path risk best_risks log_posteriors ] = minimize_risk( payoffs, loss )
%MINIMIZE_RISK Summary of this function goes here
%   Detailed explanation goes here

[ny nx] = size(payoffs);

if (nargin < 2)
    loss = @default_loss;
end

% Compute posteriors
[ log_posteriors posteriors ] = compute_posteriors(payoffs);

% Initialize DP
best = zeros([ny nx ny]);
best_risks = -ones(ny, nx);
origin = -ones(size(payoffs));

% Base cases
for y = 1:ny
    % The best model terminating at (1,y) is always just the point (1,y)
    % The cost of this model with respect to all models terminating at
    % (1,yy) is the cost this model with respect to the single model that
    % is just the point (1,yy)
    for yy = 1:ny
        best(y,1,yy) = loss(1, y, yy) * posteriors(yy,1);
    end
    best_risks(y,1) = sum(best(y,1,:));
end

% Recursive cases
for x = 2:nx
    % Sum up total probability mass for each previous position
    log_sum_marginals = zeros(ny,1);
    for z = 1:ny
        ya = max(z-1, 1);
        yb = min(z+1, ny);
        log_sum_marginals(z) = log_sum_exp(payoffs(ya:yb, x));
    end
    %report log_sum_marginals;
    
    % Minimize for each position in this column
    % y represents the sub-problem we're solving
    for y=1:ny
        
        % z is the predecessor that we're considering
        za = max(y-1, 1);
        zb = min(y+1, ny);
        for z = za:zb
            risk_comps = zeros(ny,1);
            
            % yy is the location of the reference model in the current col
            for yy = 1:ny
                updated_risk = 0;

                % zz is the location of the reference model in the previous
                % column.
                zza = max(yy-1, 1);
                zzb = min(yy+1, ny);
                for zz = zza:zzb
                    % Note: log_sum_exp(payoffs(zz-1 : zz+1, x)) ==
                    %                                log_sum_marginals(zz)
                    cond_prob = exp(payoffs(yy,x) - log_sum_marginals(zz));
                    updated_risk = updated_risk + best(z, x-1, zz)*cond_prob;
                end
                
                marginal_loss = loss(x, yy, y);
                marginal_risk = marginal_loss * posteriors(yy, x);
                risk_comps(yy) = updated_risk + marginal_risk;                
            end
            
            % Is this a better model?
            risk = sum(risk_comps);
            if ((best_risks(y,x) == -1) || (risk < best_risks(y,x)))
                best_risks(y,x) = risk;
                best(y,x,:) = risk_comps;
                origin(y,x) = z;
            end
        end
    end
end

% Find minimum in last column
risk = min(best_risks(:,nx));
last_y = find(best_risks(:,nx) == risk, 1);

% Backtrack
path = zeros(1, nx);
path(nx) = last_y;
for x = (nx-1):-1:1
    path(x) = origin(path(x+1), x+1);
end

end


function loss = default_loss(~, y_est, y_true)
loss = abs(y_est - y_true);
end
