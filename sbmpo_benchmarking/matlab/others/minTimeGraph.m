% Double Integrator Figure
% Jonathan T. Boylan
% 2023-03-07

clear
close all

%% Data

[path1, nodes1] = sbmpo_results("./plans/nodes_1.csv");
[path2, nodes2] = sbmpo_results("./plans/nodes_2.csv");
[path3, nodes3] = sbmpo_results("./plans/nodes_3.csv");

%% State space

% Path 1
x1_path = zeros(1, path1.path_size);
v1_path = zeros(1, path1.path_size);
for pt = 1:path1.path_size
   x1_path(pt) = path1.nodes(pt).state(1);
   v1_path(pt) = path1.nodes(pt).state(2);
end

x1_all = zeros(1, nodes1.buffer_size);
v1_all = zeros(1, nodes1.buffer_size);
for nd = 1:nodes1.buffer_size
    x1_all(nd) = nodes1.nodes(nd).state(1);
    v1_all(nd) = nodes1.nodes(nd).state(2);
end

% Path 2
x2_path = zeros(1, path2.path_size);
v2_path = zeros(1, path2.path_size);
for pt = 1:path2.path_size
   x2_path(pt) = path2.nodes(pt).state(1);
   v2_path(pt) = path2.nodes(pt).state(2);
end

x2_all = zeros(1, nodes2.buffer_size);
v2_all = zeros(1, nodes2.buffer_size);
for nd = 1:nodes2.buffer_size
    x2_all(nd) = nodes2.nodes(nd).state(1);
    v2_all(nd) = nodes2.nodes(nd).state(2);
end

% Plot
figure
hold on

% Path 1
plot(x1_all, v1_all, 'ob','MarkerSize',2)
plot(x1_path, v1_path,'-g','LineWidth',5)

% Path 2
plot(x2_all, v2_all, 'or','MarkerSize',2)
plot(x2_path, v2_path,'-g','LineWidth',5)

xlabel("X");
ylabel("V");
title("State space");
axis([-15 15 -5 5])

%% Time heuristic

v_opt = linspace(-15, 15, 100);
x_opt = -v_opt.*abs(v_opt)/2;

plot(x_opt, v_opt, '--k', 'LineWidth',2)