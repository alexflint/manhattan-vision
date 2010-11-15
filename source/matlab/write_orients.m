function write_orients(orients, filename, image)

[ny nx] = size(orients);
canvas = zeros(ny, nx, 3);
canvas(:,:,1) = 255 * (orients == 0);
canvas(:,:,2) = 255 * (orients == 1);
canvas(:,:,3) = 255 * (orients == 2);

if nargin>=3
  size(canvas)
  size(image)
  canvas = 0.5*canvas + 0.5*image;
end

imwrite(canvas, filename);
