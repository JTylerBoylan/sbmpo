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

% Plot V vs T
subplot(2,2,2)
plot(t,v)

% Plot U vs T
subplot(2,2,4)
plot(t,u)

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

x_sel = [];
v_sel = [];
for nd = 1:nodes.buffer_size
    if (abs(x_all(nd) - 10) + abs(v_all(nd)) < 0.001)
       node = nodes.nodes(nd) 
       idx = nd
    end
    if (abs(nodes.nodes(nd).f - 6.4028) < 0.005)
        x_sel = [x_sel, nodes.nodes(nd).state(1)];
        v_sel = [v_sel, nodes.nodes(nd).state(2)];
    end
end
plot(x_sel, v_sel, 'or', 'MarkerSize', 5);

plot(x,v,'-g','LineWidth',5)
xlabel("X");
ylabel("V");
title("State space");