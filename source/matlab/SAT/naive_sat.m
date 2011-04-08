function [ ground ] = naive_sat( clauses, num_vars )
%NAIVE_SAT Solves SAT by naive brute-force search

N = 2^num_vars;
for g = 1:N
    ground = bitand(g*ones(1,num_vars), 2.^(1:num_vars)) > 0;
    if (eval_proposition(clauses, ground))
        return;
    end
end

%No solution
ground = 0;

end
