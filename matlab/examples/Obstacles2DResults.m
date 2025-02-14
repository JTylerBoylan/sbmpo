% 2D Obstacle Path Results (Grid2D, Unicycle, Ackermann)
% Jonathan T. Boylan
% 2023-03-01

close all

goal_r = 1.0;
start_state = params.start_state;
goal = params.goal_state;

stats = sbmpo_stats("../../csv/stats.csv");
[paths, nodes] = sbmpo_results("../../csv/nodes.csv");
obstacles = sbmpo_obstacles("../../csv/obstacles.csv");

% Convert path states to points and plot
for p = 1:length(paths)

    figure('Color', [1 1 1])
    hold on
    grid on

    title(strcat("Results ", int2str(p)))
    xlabel("X (m)")
    ylabel("Y (m)")

    % Plot obstacles
    for o = 1:obstacles(end).n
        obs = [obstacles(end).x(o)-obstacles(end).r(o) obstacles(end).y(o)-obstacles(end).r(o) ...
            obstacles(end).r(o)*2 obstacles(end).r(o)*2];
        rectangle('Position', obs, 'Curvature', [1,1], 'FaceColor', 'k')
    end

    % Plot goal
    goal_point = [goal(1)-goal_r goal(2)-goal_r goal_r*2 goal_r*2];
    rectangle('Position', goal_point, 'Curvature', [1,1], 'FaceColor', 'b')

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
    
    legend('Location','northwest')

end