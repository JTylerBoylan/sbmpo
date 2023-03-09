function [paths, nodes] = sbmpo_results(results_file)
%SBMPO_RESULTS Convert results CSV to MATLAB data
%   Input: File path

    csv_matrix = readmatrix(results_file);

    paths = struct;
    p = 1;
    for line = 1:2:size(csv_matrix, 1)
        
       plan_data = csv_matrix(line,:);

       paths(p).path_size = plan_data(1);
       paths(p).num_states = plan_data(2);
       paths(p).num_controls = plan_data(3);
       i = 3;

       paths(p).nodes = struct;
       for n = 1:paths(p).path_size
            paths(p).nodes(n).generation = plan_data(i+1);
            paths(p).nodes(n).f = plan_data(i+2);
            paths(p).nodes(n).g = plan_data(i+3);
            paths(p).nodes(n).rhs = plan_data(i+4);
            i = i+4;

            paths(p).nodes(n).state = plan_data(i+1:i+paths(p).num_states);
            i = i+paths(p).num_states;
            if (n ~= paths(p).path_size)
                paths(p).nodes(n).control = plan_data(i+1:i+paths(p).num_controls);
                i = i+paths(p).num_controls;
            end
       end
       
       p = p + 1;
    end
    
    nodes = struct;
    p = 1;
    for line = 2:2:size(csv_matrix, 1)
        
       plan_data = csv_matrix(line,:);

       nodes(p).buffer_size = plan_data(1);
       nodes(p).num_states = plan_data(2);
       i = 2;

       nodes(p).nodes = struct;
       for n = 1:nodes(p).buffer_size
            nodes(p).nodes(n).generation = plan_data(i+1);
            nodes(p).nodes(n).f = plan_data(i+2);
            nodes(p).nodes(n).g = plan_data(i+3);
            nodes(p).nodes(n).rhs = plan_data(i+4);
            i = i+4;

            nodes(p).nodes(n).state = plan_data(i+1:i+nodes(p).num_states);
            i = i+nodes(p).num_states;
       end
       
       p = p + 1;
    end
end

