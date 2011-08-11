feature_spec = 'rgb,sweeps';

cases_to_load = {...
    'exeter_bursary' 5:5:35
    'exeter_mcr1' 5:5:50
    'lab_atrium2' 3:3:20
    'lab_foyer1' 5:5:45
    'lab_foyer2' 5:5:50
    'lab_kitchen1' 5:5:90
    'magd_bedroom' 5:5:100
    'magd_living' 5:5:40
    'som_corr1' 5:5:40
    };

cases = {};
for i = 1:size(cases_to_load,1)
    sequence = cases_to_load{i,1};
    indices = cases_to_load{i,2};
    
    disp(' ');
    disp(['Loading ' num2str(length(indices)) ' cases from ' sequence]);
    
    temp = dp_load_cases(sequence, indices, feature_spec);
    if i==1
        cases = temp;
    else
        cases = [cases; temp];
    end
    clear temp;
end

cases = shuffle(cases);

%cases = dp_load_cases('exeter_bursary', 5:5:35);
%cases = [cases; dp_load_cases('exeter_mcr1', 5:5:50)];
%cases = [cases; dp_load_cases('lab_atrium2', 3:3:20)];
%cases = [cases; dp_load_cases('lab_foyer1', 5:5:45)];
%cases = [cases; dp_load_cases('lab_foyer2', 5:5:50)];
%cases = [cases; dp_load_cases('lab_kitchen1', 5:5:90)];
%cases = [cases; dp_load_cases('magd_bedroom', 5:5:100)];
%cases = [cases; dp_load_cases('magd_living', 5:5:40)];
%cases = [cases; dp_load_cases('som_corr1', 5:5:40)];
