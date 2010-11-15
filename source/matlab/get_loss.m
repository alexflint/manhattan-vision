function delta = get_pixel_loss(orients_a, orients_b)

if nargin==0
  test;
  delta = [];
  return;
end

% were we passed a solution struct?
if isstruct(orients_a)
  orients_a = orients_a.orients;
end
if isstruct(orients_b)
  orients_b = orients_b.orients;
end

delta = sum(sum(orients_a ~= orients_b));


% Unit test
function test
orients_a = [ 0 1 0; 2 2 0 ];
orients_b = [ 0 2 0; 1 2 0 ];
assert(get_loss(orients_a, orients_b) == 2);
