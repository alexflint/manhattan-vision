function [ path maxpayoff ] = solve_shortest_path( payoffs )
%SOLVE_SHORTEST_PATH Summary of this function goes here
%   Detailed explanation goes here

check mmin(payoffs) >= 0;

% Initialize
[ny nx] = size(payoffs);
best = ones(ny, nx) * -inf;
srcrow = zeros(ny, nx);

% Base case
best(:,1) = payoffs(:,1);

% Recurrence cases
for x = 2:nx
    for y = 1:ny
        for py = max(1, y-1):min(ny, y+1)
            v = best(py,x-1) + payoffs(y,x);
            if (v > best(y,x))
                best(y,x) = v;
                srcrow(y,x) = py;
            end
        end
    end
end

% Backtrack
[ maxpayoff yn ] = max(best(:,nx));
path = [ zeros(1, nx-1) yn ];
for x = nx-1:-1:1
    path(x) = srcrow(path(x+1), x+1);
end

end

