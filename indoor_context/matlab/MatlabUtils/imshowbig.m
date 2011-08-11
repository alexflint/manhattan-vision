function [ h ] = imshowbig( im )
%IMSHOWBIG Utility to help display small images quickly
%   Detailed explanation goes here

zoom = max(1, 500*max(size(im,1),size(im,2)));
h = imshow(im, 'InitialMagnification', zoom);

end

