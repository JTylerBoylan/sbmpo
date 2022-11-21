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
    cost(p) = plan.cost;

end

[num_samples, sortIdx] = sort(num_samples,'ascend');
time_ms = time_ms(sortIdx);
num_nodes = num_nodes(sortIdx);
cost = cost(sortIdx);

figure
hold on
grid on
plot(num_samples,cost)
title("Number of Nodes vs. Cost")

