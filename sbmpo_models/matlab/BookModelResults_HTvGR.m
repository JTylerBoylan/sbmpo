%% Book Model Results - Rotation Control Benchmark
close all

shape = [length(HorizonTime) length(GridResolutionXY)];

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

success_filter = success_rate < 0.100;
cost_filter = cost > 7.5 | cost < 6.0;

cost(success_filter | cost_filter) = NaN;
time_ms(success_filter | cost_filter) = NaN;
num_iters(success_filter | cost_filter) = NaN;

figure
subplot(2,2,1)
hold on
grid on
contourf(X,Y,log10(time_ms))
title("Log. Computation Time (ms)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar

subplot(2,2,2)
hold on
grid on
contourf(X,Y,cost)
title("Cost")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar
plot(X(2:end,1), 0.5.*X(2:end,1)./sqrt(2), '--r', 'LineWidth',3)

subplot(2,2,3)
hold on
grid on
contourf(X,Y, log10(num_iters))
title("Log. Iterations")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar

subplot(2,2,4)
hold on
grid on
contourf(X,Y,success_rate .* 100)
title("Success Rate (%)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar

saveas(gcf, "figures/HTvsGR.fig");

time_per_iter = time_ms ./ num_iters;
