function plans = sbmpo_results(results_file)
%SBMPO_RESULTS Convert results CSV to MATLAB data
%   Input: File path

    csv_matrix = readmatrix(results_file);

    plans = struct;
    for p = 1:size(csv_matrix, 1)

       plan_data = csv_matrix(p,:);

       plans(p).path_size = plan_data(1);
       plans(p).path = plan_data(2:plans(p).path_size+1);
       i = plans(p).path_size+1;

       plans(p).buffer_size = plan_data(i+1);
       plans(p).num_states = plan_data(i+2);
       i = i + 2;

       plans(p).nodes = struct;
       for n = 1:plans(p).buffer_size
            plans(p).nodes(n).id = plan_data(i+1);
            plans(p).nodes(n).generation = plan_data(i+2);
            plans(p).nodes(n).f = plan_data(i+3);
            plans(p).nodes(n).g = plan_data(i+4);
            plans(p).nodes(n).rhs = plan_data(i+5);
            i = i+5;

            plans(p).nodes(n).state = plan_data(i+1:i+plans(p).num_states);
            i = i+plans(p).num_states;
       end

    end
end

