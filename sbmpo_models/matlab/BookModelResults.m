%% Results CSV File Parser

csv_matrix = readmatrix("../results/book_model_results.csv");

plans = struct;
for p = 1:size(csv_matrix, 1)
   plan = csv_matrix(p,:);
   
   i = 0;
   plans(p).time_ms = plan(i + 1);
   plans(p).exit_code = plan(i + 2);
   plans(p).path_size = plan(i + 3);
   i = i + 3;
   
   plans(p).path = plan(i+1:i+plans(p).path_size);
   i = i + plans(p).path_size;
   
   plans(p).num_states = plan(i+1);
   plans(p).num_controls = plan(i+2);
   plans(p).buffer_size = plan(i+3);
   i = i+3;
   
   plans(p).nodes = struct;
   for n = 1:plans(p).buffer_size
        plans(p).nodes(n).id = plan(i+1);
        plans(p).nodes(n).parent = plan(i+2);
        plans(p).nodes(n).child = plan(i+3);
        plans(p).nodes(n).generation = plan(i+4);
        plans(p).nodes(n).f = plan(i+5);
        plans(p).nodes(n).g = plan(i+6);
        i = i+6;
        
        plans(p).nodes(n).state = plan(i+1:i+plans(p).num_states);
        i = i+plans(p).num_states;
        
        plans(p).nodes(n).control = plan(i+1:i+plans(p).num_controls);
        i = i+plans(p).num_controls;
   end
end
