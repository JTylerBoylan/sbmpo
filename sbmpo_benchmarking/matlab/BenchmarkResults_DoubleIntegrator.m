% Double Integrator Results
% Jonathan T. Boylan
% 2023-03-01

close all

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

%% Path

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

figure

% Plot X vs T
subplot(2,2,[1 3])
plot(t,x)
title("Position")
xlabel("Time (s)")
ylabel("X (m)")

% Plot V vs T
subplot(2,2,2)
plot(t,v)
title("Velocity")
xlabel("Time (s)")
ylabel("V (m/s)")

% Plot U vs T
subplot(2,2,4)
plot(t,u)
title("Control")
xlabel("Time (s)")
ylabel("U (m/s^2)")

%% State space

x_all = zeros(1, nodes.buffer_size);
v_all = zeros(1, nodes.buffer_size);
for nd = 1:nodes.buffer_size
    x_all(nd) = nodes.nodes(nd).state(1);
    v_all(nd) = nodes.nodes(nd).state(2);
end

% Plot V vs X
figure
hold on
grid on
plot(x_all, v_all, 'ob','MarkerSize',2);

plot(x,v,'-g','LineWidth',5)
xlabel("X");
ylabel("V");
title("State space");
axis([-15 15 -5 5])

%% Time heuristic

v_opt = linspace(-15, 15, 100);
x_opt = -v_opt.*abs(v_opt)./(2*max(u));

plot(x_opt, v_opt, '--k')