% Double Integrator Results
% Jonathan T. Boylan
% 2023-03-01

close all

stats = sbmpo_stats("../csv/stats.csv");
[paths, nodes] = sbmpo_results("../csv/nodes.csv");

figure
hold on
grid on

for p = 1:length(paths)

    x_all = zeros(1, nodes(p).buffer_size);
    v_all = zeros(1, nodes(p).buffer_size);
    for nd = 1:nodes(p).buffer_size
        x_all(nd) = nodes(p).nodes(nd).state(1);
        v_all(nd) = nodes(p).nodes(nd).state(2);
    end

    plot(x_all, v_all, 'ob','MarkerSize',2);

end

for p = 1:length(paths)

    x_path = zeros(1, paths(p).path_size);
    v_path = zeros(1, paths(p).path_size);
    for nd = 1:paths(p).path_size
        x_path(nd) = paths(p).nodes(nd).state(1);
        v_path(nd) = paths(p).nodes(nd).state(2);
    end

    % Plot V vs X
    plot(x_path,v_path,'-g','LineWidth',5);

end

xlabel("X");
ylabel("V");
title("State space");
axis([-15 15 -5 5])

%% Time heuristic

v_opt = linspace(-5, 5, 100);
x_opt = -v_opt.*abs(v_opt)./2;

plot(x_opt, v_opt, '--k', 'LineWidth',2)