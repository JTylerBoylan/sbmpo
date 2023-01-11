%% SBMPO Results Visualizer

stats = sbmpo_stats("../results/book_model/stats.csv");
plans = sbmpo_results("../results/book_model/results.csv");
obstacles = sbmpo_obstacles("../results/book_model/obstacles.csv");

% Convert path states to points and plot
for p = 1:length(plans)

    start_x = InitialState(p,1);
    start_y = InitialState(p,2);

    goal_x = GoalState(p,1);
    goal_y = GoalState(p,2);
    goal_r = GoalThreshold(p);

    figure
    hold on
    grid on
    axis([start_x-2.5 goal_x+2.5 start_y-2.5 goal_y+2.5])

    title(strcat("Results ", int2str(p)))
    xlabel("X (m)")
    ylabel("Y (m)")

    % Plot obstacles
    for o = 1:length(obstacles.x)
        obs = [obstacles.x(o)-obstacles.r(o) obstacles.y(o)-obstacles.r(o) ...
            obstacles.r(o)*2 obstacles.r(o)*2];
        rectangle('Position', obs, 'Curvature', [1,1], 'FaceColor', 'k')
    end

    % Plot goal
    goal = [goal_x-goal_r goal_y-goal_r goal_r*2 goal_r*2];
    rectangle('Position', goal, 'Curvature', [1,1], 'FaceColor', 'b')

    % Plot all nodes
    bx = zeros(1, plans(p).buffer_size);
    by = zeros(1, plans(p).buffer_size);
    for b = 1:plans(p).buffer_size
        node = plans(p).nodes(b);
        bx(b) = node.state(1);
        by(b) = node.state(2);
    end
    plot (bx, by, 'xk', 'MarkerSize', 2)

    % Plot path
    px = zeros(1, plans(p).path_size);
    py = zeros(1, plans(p).path_size);
    for n = 1:plans(p).path_size
        node = plans(p).nodes(plans(p).path(n) + 1);
        px(n) = node.state(1);
        py(n) = node.state(2);
    end
    plot (px, py, '-g', 'LineWidth', 3)
end