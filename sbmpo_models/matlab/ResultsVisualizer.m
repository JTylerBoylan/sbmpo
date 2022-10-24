%% SBMPO Results Visualizer

clear
close all
clc

%% Extract Data 

csv_table = readtable("../results/book_model_results.csv");
times = csv_table.time_ms;
exit_codes = csv_table.exit_code;
path_sizes = csv_table.path_size;
buffer_sizes = csv_table.buffer_size;
num_states = csv_table.num_states;
num_controls = csv_table.num_controls;
path = csv_table.path;
buffer = csv_table.buffer;

%% Remove NaN

times = times(~isnan(times));
exit_codes = exit_codes(~isnan(exit_codes));
path_sizes = path_sizes(~isnan(path_sizes));
buffer_sizes = buffer_sizes(~isnan(buffer_sizes));
num_states = num_states(~isnan(num_states));
num_controls = num_controls(~isnan(num_controls));
path = path(~isnan(path));
buffer = buffer(~isnan(buffer));

%% Format into Structure