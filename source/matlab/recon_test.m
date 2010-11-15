% load some real test data
cases = dp_load_cases('lab_kitchen1', 56, 'gt');
c = cases(1);
[ny nx nf] = size(c.pixel_features);

% setup some mock weights
weights = reshape(eye(3), 9, 1);

% create objective functions
[orients, obj] = reconstruct(c, weights);

write_orients(orients, 'out/mat_soln.png');
imwrite(obj.scores(:,:,1), 'out/mat_scores0.png');
imwrite(obj.scores(:,:,2), 'out/mat_scores1.png');
imwrite(obj.scores(:,:,3), 'out/mat_scores2.png');

acc = sum(sum(orients == c.ground_truth))