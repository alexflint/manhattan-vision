function acc=get_accuracy(est_orients, gt_orients)

if nargin==0
  test;
  acc = [];
  return;
end

% were we passed a solution struct?
if isstruct(est_orients)
  est_orients = est_orients.orients;
end
if isstruct(gt_orients)
  gt_orients = gt_orients.orients;
end

ncorrect = sum(sum(est_orients == gt_orients));
[ny nx] = size(est_orients);
acc = 100 * ncorrect / (nx*ny);


% Unit test
function test

a = [ 1 2 3 ; 4 5 6 ];
b = [ 0 0 0 ; -1 -1 -1 ];
c = [ 1 2 13 ; 14 15 6 ];

check get_accuracy(a, a) == 100;
check get_accuracy(a, b) == 0;
check get_accuracy(a, c) == 50;
