% Double Integrator Results
% Jonathan T. Boylan
% 2023-03-01

close all

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

% Extract values
t = zeros(1, path.path_size);
x = zeros(1, path.path_size);
v = zeros(1, path.path_size);
u = zeros(1, path.path_size-1);
for nd = 1:path.path_size
    t(nd) = path.nodes(nd).g;
    x(nd) = path.nodes(nd).state(1);
    v(nd) = path.nodes(nd).state(2);
    if (nd < path.path_size)
        u(nd) = path.nodes(nd).control(1);
    else
        u(nd) = NaN;
    end
end

% Plot U vs T
figure
plot(t,u)

% Plot V vs X
figure
hold on
grid on
plot(x,v)