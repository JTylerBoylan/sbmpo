%% Book Model Results - Rotation Control Benchmark
close all

shape = [50 50];

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

size = length(plans);
time_ms = zeros(1, size);
num_iters = zeros(1, size);
cost = zeros(1, size);
exit_code = zeros(1, size);

for p = 1:size
    
    plan = plans(p);
    time_ms(p) = plan.time_ms;
    num_iters(p) = plan.buffer_size / NumberOfSamples(p);
    cost(p) = plan.cost;
    exit_code(p) = plan.exit_code;

end

X = reshape(SampleHorizonTime, shape);
Y = repmat(GridResolution(:,1)', [shape(1) 1]);

time_ms = reshape(time_ms, shape);
num_iters = reshape(num_iters, shape);
cost = reshape(cost, shape);
exit_code = reshape(exit_code, shape);

figure
subplot(2,2,1)
hold on
grid on
contourf(X,Y,log(time_ms))
title("Log. Computation Time (ms)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon")

subplot(2,2,2)
hold on
grid on
contourf(X,Y,cost)
title("Cost (m)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon")

subplot(2,2,3)
hold on
grid on
contourf(X,Y,log(num_iters))
title("Log. Iterations")
ylabel("Grid Resolution")
xlabel("Sampling Horizon")

subplot(2,2,4)
hold on
grid on
contourf(X,Y,exit_code)
title("Exit Code")
ylabel("Grid Resolution")
xlabel("Sampling Horizon")
