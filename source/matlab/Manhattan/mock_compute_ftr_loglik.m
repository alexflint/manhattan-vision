function [ loglik J_loglik ] = mock_compute_model_loglik( wts )
%MOCK_COMPUTE_MODEL_LOGLIK Summary of this function goes here
%   Detailed explanation goes here

% Mock features
nx = 5;
ny = 5;
payoffs = zeros(ny,nx,2,length(wts));
for i=1:length(wts)
    payoffs(:,:,1,i) = reshape([i*10 : i*10+nx*ny-1], [ny ny]);
    payoffs(:,:,2,i) = reshape([i*25 : i*25+nx*ny-1], [ny ny]);
end
path = [2 1 1 2 3];
orients = [2 2 1 1 1];

% Sum logit values over path
logit = 0;
J_logit = zeros(size(wts));
for x = 1:nx
    f = reshape(payoffs(path(x), x, orients(x), :), size(wts));
    y = sum(f .* wts);
    logit = logit + log(1 + exp(-y));
    J_logit = J_logit + f * (exp(-y) / (1+exp(-y)));
end

loglik = -nx*log(norm(wts)) - logit;
J_loglik = -nx*wts/(norm(wts)*norm(wts)) + J_logit;

end
