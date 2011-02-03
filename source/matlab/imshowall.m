function imshowall(varargin)
% IMSHOWALL show images simultaneously

ims = cell(0);

for i = 1:length(varargin)
  x = varargin{i};
  if iscell(x)
      for j=1:length(x)
          ims{length(ims)+1} = x{j};
      end
  else
    d = size(x);
    check length(d)>=2 && length(d)<=3;
    for j=1:size(x,3)
        ims{length(ims)+1} = x(:,:,j);
    end
  end
end

n = length(ims);
nc = ceil(sqrt(n));
nr = ceil(n/nc);

for i=1:length(ims)
  subplot(nr,nc,i), subimage(ims{i});
end
