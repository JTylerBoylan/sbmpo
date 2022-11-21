%% Book Model Results - Samples Benchmark
close all

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

size = length(plans);

num_samples = zeros(1, size);
num_linear = zeros(1, size);
num_rotation = zeros(1, size);
time_ms = zeros(1, size);
num_nodes = zeros(1, size);
cost = zeros(1, size);

for p = 1:size
    
    plan = plans(p);

    num_samples(p) = NumberOfSamples(p);
    num_linear(p) = length(cell2mat(V(LinearControls,p)));
    num_rotation(p) = length(cell2mat(V(RotationControls,p)));
    time_ms(p) = plan.time_ms;
    num_nodes(p) = plan.buffer_size;
    cost(p) = plan.cost;

end

[num_linear, sortIdx] = sort(num_linear,'ascend');
num_linear = reshape(num_linear, [4 3]);

num_samples = num_samples(sortIdx);
num_samples = reshape(num_samples, [4 3]);

num_rotation = num_rotation(sortIdx);
num_rotation = reshape(num_rotation, [4 3]);

time_ms = time_ms(sortIdx);
time_ms = reshape(time_ms, [4 3]);

num_nodes = num_nodes(sortIdx);
num_nodes = reshape(num_nodes, [4 3]);

cost = cost(sortIdx);
cost = reshape(cost, [4 3]);

figure
hold on
grid on
contourf(num_linear,num_rotation,time_ms)
title("Time (ms)")
ylabel("Number of Rotation Controls");
xlabel("Number of Linear Controls");

figure
hold on
grid on
contourf(num_linear,num_rotation,num_nodes)
title("Number of Nodes")
ylabel("Number of Rotation Controls");
xlabel("Number of Linear Controls");

figure
hold on
grid on
contourf(num_linear,num_rotation,cost)
title("Cost")
ylabel("Number of Rotation Controls");
xlabel("Number of Linear Controls");

