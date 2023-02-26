function stats = sbmpo_stats(stats_file)
%SBMPO_STATS Convert stats CSV to MATLAB data
%   Input: File path

    csv_matrix = readmatrix(stats_file);

    stats = struct;
    for p = 1:size(csv_matrix, 1)

       stat_data = csv_matrix(p,:);

       stats(p).time_us = stat_data(1);
       stats(p).exit_code = stat_data(2);
       stats(p).iterations = stat_data(3);
       stats(p).cost = stat_data(4);
       stats(p).buffer_size = stat_data(5);
       stats(p).success_rate = stat_data(6);

    end

end

