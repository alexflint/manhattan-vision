function delta = get_pixel_loss(model_a, model_b)

if nargin==0
  test;
  delta = [];
  return;
end

% were we passed a solution struct?
if isstruct(model_a)
  orients_a = model_a.orients;
else
  orients_a = model_a;
end

if isstruct(model_b)
  orients_b = model_b.orients;
else
  orients_b = model_b;
end

delta = sum(sum(orients_a ~= orients_b));


% Unit test
function test
orients_a = [ 0 1 0; 2 2 0 ];
orients_b = [ 0 2 0; 1 2 0 ];
check get_pixel_loss(orients_a, orients_b) == 2;
