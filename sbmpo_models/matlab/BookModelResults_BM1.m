%% Book Model Results - Samples Benchmark
close all

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

size = length(plans);

num_samples = zeros(1, size);
time_ms = zeros(1, size);
num_nodes = zeros(1, size);
cost = zeros(1, size);

for p = 1:size
    
    plan = plans(p);

    num_samples(p) = NumberOfSamples(p);
    time_ms(p) = plan.time_ms;
    num_nodes(p) = plan.buffer_size;
    for n = 1:length(plan.nodes)
       if plan.nodes(n).id == plan.path(end)
           cost(p) = plan.nodes(n).g;
       end
    end

end

figure
hold on
grid on
plot(num_samples,time_ms)
plot(num_samples,num_nodes)
title("Number of Nodes vs. Number of Samples")

