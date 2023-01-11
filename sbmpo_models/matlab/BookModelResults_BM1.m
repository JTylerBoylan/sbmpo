%% Book Model Results - Rotation Control Benchmark
close all

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

X = GridResolution(:,1)';

figure
subplot(4,1,1)
hold on
grid on
bar(X,time_ms)
ylabel("Computation Time (ms)")

subplot(4,1,2)
hold on
grid on
bar(X,cost)
ylabel("Cost (m)")

subplot(4,1,3)
hold on
grid on
bar(X,num_iters)
ylabel("Iterations")

subplot(4,1,4)
hold on
grid on
bar(X,success_rate)
ylabel("Success Rate (%)")
xlabel("Grid Resolution")

sgtitle('Benchmark 1')

saveas(gcf,'figures/benchmark1.fig')
