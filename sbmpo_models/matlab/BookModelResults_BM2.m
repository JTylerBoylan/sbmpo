%% Book Model Results - Rotation Control Benchmark
close all

shape = [40 46];

stats = sbmpo_stats("../results/book_model/stats.csv");

size = length(stats);
time_ms = zeros(1, size);
num_iters = zeros(1, size);
cost = zeros(1, size);
success_rate = zeros(1, size);

for p = 1:size
    
    stat = stats(p);
    time_ms(p) = stat.time_ms;
    num_iters(p) = stat.buffer_size / NumberOfSamples(p);
    cost(p) = stat.cost;
    success_rate(p) = stat.success_rate;

end

X = reshape(SampleHorizonTime, shape);
Y = repmat(GridResolution(:,1)', [shape(1) 1]);

time_ms = reshape(time_ms, shape);
num_iters = reshape(num_iters, shape);
cost = reshape(cost, shape);
success_rate = reshape(success_rate, shape);

figure
subplot(2,2,1)
hold on
grid on
contourf(X,Y,time_ms)
title("Computation Time (ms)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")

subplot(2,2,2)
hold on
grid on
contourf(X,Y,cost)
title("Cost (m)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")

subplot(2,2,3)
hold on
grid on
contourf(X,Y, num_iters)
title("Iterations")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")

subplot(2,2,4)
hold on
grid on
contourf(X,Y,success_rate)
title("Success Rate (%)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")

saveas(gcf, "figures/benchmark2.fig");
