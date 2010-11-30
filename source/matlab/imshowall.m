function imshowall(varargin)
% IMSHOWALL show images simultaneously

ims = cell(0);

n = 0;
for i = 1:length(varargin)
  x = varargin{i};
  d = size(x);
  check length(d)>=2 && length(d)<=3;
  for j=1:size(x,3)
    n = n+1;
    ims{n} = x(:,:,j);
  end
end

nc = ceil(sqrt(n));
nr = ceil(n/nc);

for i=1:length(ims)
  subplot(nr,nc,i), subimage(ims{i});
end

    