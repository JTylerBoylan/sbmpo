%% SBMPO Results Visualizer

plans = sbmpo_results("../results/book_model_results.csv");
obstacles_csv = readmatrix("../results/obstacles.csv");

% Set goal
goal = [GoalState(1:2), 0.3];

% Convert path states to points and plot
for p = 1:length(plans)

    figure
    hold on
    grid on
    axis([-2.5 7.5 -2.5 7.5])

    title(strcat("Results ", int2str(p)))
    xlabel("X (m)")
    ylabel("Y (m)")

    % Plot obstacles
    if (~isempty(obstacles_csv))
        obsc = obstacles_csv(p,:);
        obstacles = reshape(obsc(2:end), [obsc(1) 3])';
        obs = [obstacles(:,1:2)-obstacles(:,3) obstacles(:,3).*2 obstacles(:,3).*2];
        for o = 1:length(obstacles)
            rectangle('Position',obs(o, :), 'Curvature', [1,1], 'FaceColor', 'k')
        end
    end

    % Plot goal
    gol = [goal(1:2)-goal(3) goal(3)*2 goal(3)*2];
    rectangle('Position', gol, 'Curvature', [1,1], 'FaceColor', 'b')

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