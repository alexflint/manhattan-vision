function av_acc=evaluate(cases, weights)

assert(nargin == 2);

acc = zeros(length(cases),1);
for i = 1:length(cases)
  soln = reconstruct(cases(i), weights);
  acc(i) = get_accuracy(soln, cases(i).ground_truth);
end

disp('\n\n');
for i = 1:length(cases)
  disp([ 'Frame ' num2str(cases(i).frame_id) ':      Accuracy=' num2str(acc(i)) '%' ]);
end

av_acc = mean(acc);
disp('---');
disp(['Average accuracy:       ' num2str(av_acc) '%']);
disp('---');
