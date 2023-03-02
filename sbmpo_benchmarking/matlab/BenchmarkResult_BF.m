% Branchout Factor Comparison Results
% Jonathan T. Boylan
% 2023-03-01

close all

stats = sbmpo_stats("../csv/stats.csv");

max_branchout = max(NumberOfSamples);
num_samples = 1:max_branchout;
time_ms = zeros(1, max_branchout);
buffer_size = zeros(1, max_branchout);
cost = zeros(1, max_branchout);
success_rate = zeros(1, max_branchout);
num_runs = zeros(1,max_branchout);

for s = 1:length(stats)
    ns = NumberOfSamples(s);
    time_ms(ns) = time_ms(ns) + stats(s).time_ms;
    buffer_size(ns) = buffer_size(ns) + stats(s).buffer_size;
    cost(ns) = cost(ns) + stats(s).cost;
    success_rate(ns) = success_rate(ns) + stats(s).success_rate;
    num_runs(ns) = num_runs(ns) + 1;
end

rm_empty = num_runs ~= 0;
num_runs = num_runs(rm_empty);
num_samples = num_samples(rm_empty);
time_ms = time_ms(rm_empty) ./ num_runs;
buffer_size = buffer_size(rm_empty) ./ num_runs;
cost = cost(rm_empty) ./ num_runs;
success_rate = success_rate(rm_empty) ./ num_runs;

figure1 = figure('Color', [1 1 1]);
subplot1 = subplot(3,1,1);
hold on
grid on
plot(num_samples, time_ms, '-b', 'LineWidth', 3)
plot(num_samples, time_ms, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 10)
ylabel("Time (ms)")
set(subplot1,'XTick',[9 15 21 27]);

subplot2 = subplot(3,1,2);
hold on
grid on
plot(num_samples, buffer_size, '-b', 'LineWidth', 3)
plot(num_samples, buffer_size, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 10)
ylabel("Number of Nodes")
set(subplot2,'XTick',[9 15 21 27]);

subplot3 = subplot(3,1,3);
hold on
grid on
plot(num_samples, cost, '-b', 'LineWidth', 3)
plot(num_samples, cost, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 10)
xlabel("Branchout Factor")
ylabel("Cost (m)")
set(subplot3,'XTick',[9 15 21 27]);
