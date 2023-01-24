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
cost_filter = cost > 100 | cost < 1;

cost(success_filter | cost_filter) = NaN;
time_ms(success_filter | cost_filter) = NaN;
num_iters(success_filter | cost_filter) = NaN;

figure
hold on
grid on
contourf(X,Y,time_ms)
title("Computation Time (ms)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar
%plot([0.3 0.7], 0.5.*[0.3 0.7]./sqrt(2), '-r', 'LineWidth',3)
%plot([0.5 0.5], [0.1 0.25], '--k', 'LineWidth', 3)
%saveas(gcf, 'figures/time_050_01768.fig')

figure
hold on
grid on
contourf(X,Y,cost)
title("Cost")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar
%plot([0.3 0.7], 0.5.*[0.3 0.7]./sqrt(2), '-r', 'LineWidth',3)
%plot([0.5 0.5], [0.1 0.25], '--k', 'LineWidth', 3)
%saveas(gcf, 'figures/cost_050_01768.fig')

figure
hold on
grid on
contourf(X,Y, num_iters)
title("Iterations")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
colorbar
%plot([0.3 0.7], 0.5.*[0.3 0.7]./sqrt(2), '-r', 'LineWidth',3)
%plot([0.5 0.5], [0.1 0.25], '--k', 'LineWidth', 3)
%saveas(gcf, 'figures/iter_050_01768.fig')

figure1 = figure('Color',[1 1 1]);
axes1 = axes('Parent', figure1);
hold(axes1,'on');
contourf(X,Y,success_rate .* 100)
title("Success Rate (%)")
ylabel("Grid Resolution")
xlabel("Sampling Horizon Time")
box(axes1,'on')
grid(axes1,'on')
hold(axes1,'off')
set(axes1, 'FontSize', 64)
colorbar
%plot([0.3 0.7], 0.5.*[0.3 0.7]./sqrt(2), '-r', 'LineWidth',3)
%plot([0.5 0.5], [0.1 0.25], '--k', 'LineWidth', 3)

time_per_iter = time_ms ./ num_iters;
