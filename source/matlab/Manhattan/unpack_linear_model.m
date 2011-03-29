function [ model verts ncorners noccl ] = unpack_linear_model( linear_theta, geometry )

[ K d ] = size(linear_theta);
check d == 3;

xs = zeros(1, K);
ys = zeros(1, K);
as = zeros(1, K);

% We assume the horizon is at y=1
vpts = [ geometry.vl 1 1
         geometry.vr 1 1 ];

% Initialize vertices
verts = zeros(K*2-1, 2);
ncorners = 0;
noccl = 0;

for i = 1:K
    xs(i) = linear_theta(i, 1);
    ys(i) = linear_theta(i, 2);
    as(i) = linear_theta(i, 3);

    % Sanity check
    check xs(i) >= 0 && xs(i) <= 1;
    check ys(i) >= 0 && ys(i) <= 1;
    check as(i) == 1 || as(i) == 2;
    
    % For all columns other than the first, constrain Yi to the feasible
    % region
    if (i > 1)
        % Extrapolate y(i-1) to this column
        yprime = extrapolate_wall(xs(i), xs(i-1), ys(i-1), as(i-1), vpts);
        %prev_p = [ xs(i-1) ys(i-1) 1 ];
        %prev_vpt = vpts(as(i-1), :);
        %col = [ -1 0 xs(i) ];
        %wall = cross( prev_vpt , prev_p );
        %isct = cross(col, wall);
        %yprime = isct(2) / isct(3);

        % Append vertex
        verts(i*2-2, :) = [ xs(i) yprime ];

        % Decide bounds for this Y
        % this assumes that we are only working *below* the horizon
        yi_can_be_less = occlusion_is_valid(xs(i), as(i-1), as(i), 1, geometry);
        yi_can_be_greater = occlusion_is_valid(xs(i), as(i-1), as(i), -1, geometry);
        if (yi_can_be_less)
            ymin = 0;
        else
            ymin = yprime;
        end
        if (yi_can_be_greater)
            ymax = 1;
        else
            ymax = yprime;
        end

        % Decide yi
        ys(i) = (1-ys(i))*ymin + ys(i)*ymax;
        
        % TODO: better way to deal with normal/occluding corners
        if (ys(i) == yprime)
            ncorners = ncorners + 1;
        else
            noccl = noccl + 1;
        end
            
    end
    
    % Append vertex (must come after the one added during the IF)
    verts(i*2-1, :) = [ xs(i) ys(i) ];
end

% Append the last vertex
yfinal = extrapolate_wall(1, xs(K), ys(K), as(K), vpts);
verts(K*2, :) = [1 yfinal];

% Construct the final model
model = make_model(xs', ys', as');


function [ yprime ] = extrapolate_wall(x, xprev, yprev, aprev, vpts)

% Extrapolate a wall to a particular column
prev_p = [ xprev yprev 1 ];
prev_vpt = vpts(aprev, :);
col = [ -1 0 x ];
wall = cross( prev_vpt , prev_p );
isct = cross(col, wall);
yprime = isct(2) / isct(3);

end

end
