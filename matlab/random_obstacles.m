function random_obstacles(filename, params, runs)

    for run = 1:runs
        
        n = floor(rand() * (params.max_n - params.min_n) + params.min_n);
        x = rand(1,n) * (params.max_x - params.min_x) + params.min_x;
        y = rand(1,n) * (params.max_y - params.min_y) + params.min_y;
        r = rand(1,n) * (params.max_r - params.min_r) + params.min_r;

        obs = [x; y; r];
        obstacles =[n  reshape(obs, [1 3*n])];

        if (run == 1)
            writematrix(obstacles, filename, 'Delimiter', ',');
        else
            writematrix(obstacles, filename, 'Delimiter', ',', 'WriteMode', 'append');
        end
    
    end
    
end