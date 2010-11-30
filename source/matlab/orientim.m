function canvas=orientim(orients, bg)
% Generate a RGB canvas from an orientation map

if isstruct(orients)
  orients = orients.orients;
end

if nargin>=2 && isstruct(bg)
  bg = imread(bg.image_file);
end

[ny nx] = size(orients);
canvas = zeros(ny, nx, 3);
canvas(:,:,1) = 255 * (orients == 0);
canvas(:,:,2) = 255 * (orients == 1);
canvas(:,:,3) = 255 * (orients == 2);

% Blend with background image if one was provided
if nargin>=2
  canvas = uint8(0.5*canvas + 0.5*double(bg));
end
