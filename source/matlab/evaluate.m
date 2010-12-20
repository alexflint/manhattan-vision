function av_acc=evaluate(cases, weights, generate_images)

check nargin >= 2;
default generate_images = 0;

acc = zeros(length(cases),1);
for i = 1:length(cases)
  soln = reconstruct(cases(i), weights);
  acc(i) = get_accuracy(soln, cases(i).ground_truth);
  if generate_images
    outfile = sprintf('out/frame%03d_orients.png', cases(i).frame_id);
    %imwrite(orientim(soln.orients, cases(i).frame.image), outfile);
    imwrite(orientim(soln.orients), outfile);
    disp(['Accuracy=' num2str(acc(i)) '%' ]);
    disp(['Output ' outfile]);
  end
end

for i = 1:length(cases)
  disp([ 'Frame ' num2str(cases(i).frame_id) ':      Accuracy=' num2str(acc(i)) '%' ]);
end

av_acc = mean(acc);
disp('---');
disp(['Average accuracy:       ' num2str(av_acc) '%']);
disp('---');
