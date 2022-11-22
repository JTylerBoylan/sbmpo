%% Book Model Results - Linear Control Benchmark
close all

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

size = length(plans);

time_ms = zeros(1, size);
num_iters = zeros(1, size);
cost = zeros(1, size);
nodes_per_iter = [27 18 18 18 9 9 9];

for p = 1:size
    
    plan = plans(p);

    time_ms(p) = plan.time_ms;
    num_iters(p) = plan.buffer_size / nodes_per_iter(p);
    cost(p) = plan.cost;

end

X = categorical({'All','0.1,0.3','0.1,0.5','0.3,0.5','0.1','0.3','0.5'});

figure
subplot(3,1,1)
hold on
grid on
bar(X,time_ms)
ylabel("Computation Time (ms)")

subplot(3,1,2)
hold on
grid on
bar(X,cost)
ylabel("Cost (m)")

subplot(3,1,3)
hold on
grid on
bar(X,num_iters)
ylabel("Iterations")
xlabel("Control")

sgtitle('Benchmark 2')

saveas(gcf,'figures/benchmark2.png')
