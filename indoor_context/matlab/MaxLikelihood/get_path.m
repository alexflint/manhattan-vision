function [ path mask ] = get_path( verts, ny, nx )
%GET_PATH Summary of this function goes here
%   Detailed explanation goes here

path = zeros(1, nx);
mask = zeros(ny, nx);

i = 1;
for x = 1:nx
    % Move along the list of vertices
    while (x/nx > verts(i*2, 1))    % recall, vertices come in [begin end] pairs
        i = i + 1;
        check i*2 <= size(verts, 1);
    end
    
    % Compute location at this column
    left = verts(i*2-1, :) .* [nx ny];
    right = verts(i*2, :) .* [nx ny];
    col = [ -1 0 x ];
    isct = cross(col, cross([left 1], [right 1]));
    
    % Update path and mask
    path(x) = isct(2) / isct(3);
    
    y = round(path(x));
    check y >= 1;
    check y < ny;
    mask(y,x) = 1;
end

end

