function [ value ] = eval_proposition( clauses, ground )
%EVAL_PROPOSITION Evaluates a CNF proposition

value = 1;
for i = 1:length(clauses)
    clause = clauses{i};
    clause_value = 0;
    for atom = clause;
        if atom >= 0
            v = ground(atom);
        else
            v = ~ground(-atom);
        end
        if v
            clause_value = 1;
            break;
        end
    end
    if ~clause_value
        value = 0;
        break;
    end
end

