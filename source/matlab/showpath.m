function [ h im ] = showpath( path, payoffs )
%SHOWPATH Summary of this function goes here
%   Detailed explanation goes here

[ny nx] = size(payoffs);
check length(path) == nx;
check max(path) <= ny;

% Create red stripe for the path
mask = zeros(ny,nx);
mask( path + [0:ny:ny*nx-1] ) = 1;
pathim = zeros(ny,nx,3);
pathim(:,:,1) = mask*255;

if (nargin > 1)
    % Create an RGB image of the payoffs
    payoffim(:,:,1) = payoffs*255;
    payoffim(:,:,2) = payoffs*255;
    payoffim(:,:,3) = payoffs*255;

    % Blend
    im = uint8(0.5*payoffim + 0.5*pathim);
else
    im = uint8(pathim);
end
    
zoom = max(1, 500*max(nx,ny));
h = imshow(im, 'InitialMagnification', zoom);

end
