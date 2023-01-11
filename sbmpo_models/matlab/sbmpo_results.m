function plans = sbmpo_results(results_file)
%SBMPO_RESULTS Convert results CSV to MATLAB data
%   Input: File path

    csv_matrix = readmatrix(results_file);

    plans = struct;
    for p = 1:size(csv_matrix, 1)

       plan_data = csv_matrix(p,:);
       save_type = plan_data(1);

       i = 1;

       if (bitand(save_type,0b1))
           plans(p).time_ms = plan_data(i + 1);
           plans(p).exit_code = plan_data(i + 2);
           plans(p).cost = plan_data(i + 3);
           plans(p).buffer_size = plan_data(i + 4);
           plans(p).success_rate = plan_data(i + 5);
           i = i + 5;
       end

       if (bitand(save_type,0b10))
           plans(p).path_size = plan_data(i + 1);
           i = i + 1;
           plans(p).path = plan_data(i+1:i+plans(p).path_size);
           i = i + plans(p).path_size;
       end

       if(bitand(save_type,0b110))
           plans(p).num_states = plan_data(i+1);
           %plans(p).num_controls = plan_data(i+2);
           i = i+1;
           
           N = plans(p).buffer_size;
           if (bitand(save_type,0b10) && ~bitand(save_type,0b100))
              N = plans(p).path_size; 
           end

           plans(p).nodes = struct;
           for n = 1:N
                plans(p).nodes(n).id = plan_data(i+1);
                plans(p).nodes(n).generation = plan_data(i+2);
                plans(p).nodes(n).f = plan_data(i+3);
                plans(p).nodes(n).g = plan_data(i+4);
                plans(p).nodes(n).rhs = plan_data(i+5);
                i = i+5;

                plans(p).nodes(n).state = plan_data(i+1:i+plans(p).num_states);
                i = i+plans(p).num_states;

                %plans(p).nodes(n).control = plan_data(i+1:i+plans(p).num_controls);
                %i = i+plans(p).num_controls;
           end
       end

    end
end

