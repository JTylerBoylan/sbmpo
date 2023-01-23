%% SBMPO Results Visualizer

stats = sbmpo_stats("../results/book_model/stats.csv");

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

figure
plot(num_samples, time_ms)
title("Time")

figure
plot(num_samples, buffer_size)
title("Buffer Size")

figure
plot(num_samples, cost)
title("Cost")

figure
plot(num_samples, success_rate)
title("Success Rate")