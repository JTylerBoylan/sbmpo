function plans = results(path)
%RESULTS Convert results CSV file to matlab structure
%   Usage: results("path/to/csv")

    % Extract Data 
    csv_table = readtable(path);
    times = csv_table.time_ms;
    exit_codes = csv_table.exit_code;
    path_sizes = csv_table.path_size;
    buffer_sizes = csv_table.buffer_size;
    num_states = csv_table.num_states;
    num_controls = csv_table.num_controls;
    path = csv_table.path;
    buffer = csv_table.buffer;
    
    % Format into Structure
    plans = struct;
    
    i = 1;
    j = 0;
    for p = 1:length(times(~isnan(times)))
        plans(p).time_ms = times(p);
        plans(p).exit_code = exit_codes(p);
        plans(p).path_size = path_sizes(p);
        plans(p).buffer_size = buffer_sizes(p);
        plans(p).num_states = num_states(p);
        plans(p).num_controls = num_controls(p);
        plans(p).path = zeros(1, path_sizes(p));
        for a = 1:path_sizes(p)
            plans(p).path(a) = path(j + a);
        end
        j = j + path_sizes(p);
        plans(p).nodes = struct;
        for n = 1:buffer_sizes(p)
            plans(p).nodes(n).id = buffer(i + 0);
            plans(p).nodes(n).parent = buffer(i + 1);
            plans(p).nodes(n).child = buffer(i + 2);
            plans(p).nodes(n).generation = buffer(i + 3);
            plans(p).nodes(n).f = buffer(i + 4);
            plans(p).nodes(n).g = buffer(i + 5);
            plans(p).nodes(n).state = zeros(num_states(p), 1);
            plans(p).nodes(n).control = zeros(num_controls(p), 1);
            for s = 1:num_states(p)
                plans(p).nodes(n).state(s) = buffer(i + 5 + s);
            end
            for c = 1:num_controls(p)
                plans(p).nodes(n).control(c) = buffer(i + 5 + num_states(p) +  c);
            end
            i = i + 6 + num_states(p) + num_controls(p);
        end
    end
end

