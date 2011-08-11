function w=mock_weights(casedata)

[ny nx nf] = size(casedata.pixel_features);
[gny gnx ns] = size(casedata.wall_features);

pix_w = floor(randn(3, nf)*5);
wall_w = floor(randn(2, ns)*5);
w = make_weights(pix_w, wall_w, floor(rand()*5), floor(rand()*5));
