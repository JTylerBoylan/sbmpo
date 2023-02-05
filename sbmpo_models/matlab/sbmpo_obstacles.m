function obstacles = sbmpo_obstacles(obstacles_file)
%SBMPO_STATS Convert stats CSV to MATLAB data
%   Input: File path

    csv_matrix = readmatrix(obstacles_file, "NumHeaderLines", 0);

    obstacles = struct;
    for p = 1:size(csv_matrix, 1)

       obs_data = csv_matrix(p,:);

       num_obs =  obs_data(1);
       obstacles(p).n = num_obs;
       obstacles(p).x = zeros(1, num_obs);
       obstacles(p).y = zeros(1, num_obs);
       obstacles(p).r = zeros(1, num_obs);

       for o = 1:num_obs
           obstacles(p).x(o) = obs_data(3*o - 1);
           obstacles(p).y(o) = obs_data(3*o);
           obstacles(p).r(o) = obs_data(3*o + 1);
       end

    end

end

