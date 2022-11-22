%% Book Model Results - Rotation Control Benchmark
close all

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

size = length(plans);
time_ms = zeros(1, size);
num_iters = zeros(1, size);
cost = zeros(1, size);

for p = 1:size
    
    plan = plans(p);
    time_ms(p) = plan.time_ms;
    num_iters(p) = plan.buffer_size / NumberOfSamples(p);
    cost(p) = plan.cost;

end

X = categorical({'1','3','5','7','13','15','17','35','37','57','135','137','157','357','1357'});

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
xlabel("Rotational Control")

sgtitle('Benchmark 3')

saveas(gcf,'figures/benchmark3.png')
