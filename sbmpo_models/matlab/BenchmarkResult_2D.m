%% SBMPO Results Visualizer
close all

stats = sbmpo_stats("../benchmarking/stats.csv");
[paths, nodes] = sbmpo_results("../benchmarking/nodes.csv");
obstacles = sbmpo_obstacles("../benchmarking/obstacles.csv");

%rrt_results = readmatrix('docs/result_obstacles.txt','Delimiter',' ');
%rrt_results = readmatrix('docs/result_large.txt','Delimiter',' ');

% Convert path states to points and plot
for p = 1:length(paths)

    start = V(InitialState, p);
    goal = V(GoalState, p);
    goal_r = V(GoalThreshold, p);

    figure('Color', [1 1 1])
    hold on
    grid on
    axis([-5 5 -5 5])

    %title(strcat("Results ", int2str(p)))
    xlabel("X (m)")
    ylabel("Y (m)")

    % Plot obstacles
    for o = 1:obstacles(p).n
        obs = [obstacles(p).x(o)-obstacles(p).r(o) obstacles(p).y(o)-obstacles(p).r(o) ...
            obstacles(p).r(o)*2 obstacles(p).r(o)*2];
        rectangle('Position', obs, 'Curvature', [1,1], 'FaceColor', 'k')
    end

    % Plot goal
    goal = [goal(1)-goal_r goal(2)-goal_r goal_r*2 goal_r*2];
    rectangle('Position', goal, 'Curvature', [1,1], 'FaceColor', 'b')

    % Plot all nodes
    nx = zeros(1, nodes(p).buffer_size);
    ny = zeros(1, nodes(p).buffer_size);
    for n = 1:nodes(p).buffer_size
        node = nodes(p).nodes(n);
        nx(n) = node.state(1);
        ny(n) = node.state(2); 
    end
    plot (nx, ny, 'ob', 'MarkerSize', 2, 'HandleVisibility', 'off')
    
    % Plot path
    px = zeros(1, paths(p).path_size);
    py = zeros(1, paths(p).path_size);
    for n = 1:paths(p).path_size
        node = paths(p).nodes(n);
        px(n) = node.state(1);
        py(n) = node.state(2);
    end
    plot (px, py, '-g', 'LineWidth', 5, 'DisplayName', 'SBMPO')
    plot (px, py, 'ob', 'MarkerSize', 5, 'HandleVisibility', 'off')

    %rrt_x = rrt_results(:,1);
    %rrt_y = rrt_results(:,2);
    %plot(rrt_x, rrt_y, 'LineWidth', 5, 'DisplayName', 'RRT')
    
    legend('Location','northwest')
    
    saveas(gcf, strcat("figures/rrt_obstacle_comp/result", int2str(p), ".png"))

end