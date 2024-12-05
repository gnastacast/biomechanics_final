function [vels, liftoff, landing, fp, times] = analyze_data(t, xy, xyFP, ....
    n_feet, min_vel, min_pos, smoothing_time, debug_plot)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    if debug_plot
        plot(t, xy(:,2), Color='#77ac30', LineWidth=3);
        hold on
    end
    
    mask = zeros(size(t));
    times = [];
    fp_guess = [];
    for leg=1:n_feet
        leg_cols = size(xyFP,2) / n_feet;
        xyFP_leg = xyFP(:, (leg-1) * leg_cols + 1 : leg * leg_cols);
        [next_fp_guess, next_times, next_mask] = find_foot_placements( ...
            xyFP_leg, t, min_vel, min_pos, smoothing_time, debug_plot);
        mask = or(mask, next_mask);
        times = [times; next_times];
        fp_guess = [fp_guess; next_fp_guess];
    end

    [times, order] = sort(times, 1);
    fp_guess = fp_guess(order,:);
    
    if debug_plot
        scatter(t(not(mask)), xy(not(mask),2))
        if ~isempty(fp_guess)
            % scatter(times, ones(size(times)))
            scatter(times, fp_guess(:,2),'red','filled','o','LineWidth',3)
        end
    end

    vels = [];
    liftoff = [];
    landing = [];
    fp = [];
    n_times = size(times,1);
    for i = 0:n_times
        if i == 0
            t_start = t(1);
        else
            t_start = times(i);
        end
        if i < size(times,1)
            t_end = times(i + 1);
        else
            t_end = t(end);
        end
        sub_mask = and(not(mask), and(t >= t_start, t <= t_end));
        if sum(sub_mask) < 3
            continue
        end
        sub_time = t(sub_mask);
        t_start = sub_time(1);
        t_end = sub_time(end);
        py = polyfit(sub_time, xy(sub_mask, 2), 2);
        px = polyfit(sub_time, xy(sub_mask, 1), 1);
        if(debug_plot)
            plot(sub_time, polyval(py, sub_time), 'k', LineWidth=2);
        end
        
        yvel =  2 * sub_time(1) * py(1) + py(2);
        vels = [vels; px(1), yvel];
        liftoff = [liftoff; [polyval(px, t_start), polyval(py, t_start)]];
        if i < size(times,1)
            foot_xy = [fp_guess(i+1,1) - polyval(px, t_end), ...
                       fp_guess(i+1,2) - polyval(py, t_end)];
            landing = [landing; [polyval(px, t_end), polyval(py, t_end)]];
            fp = [fp; foot_xy];
        end

    end
    % figure()
    % scatter(foot_placement(:,1) + landings(:,1), foot_placement(:,2) + landings(:,2))
    if debug_plot
        legend();
        hold off
        axis equal
    end

end

function [fp_guess, times, mask] = find_foot_placements(xy_foot, t, min_vel, min_pos, smoothing_time, debug_plot)
    dt = gradient(t);
    ttable = timetable(seconds(t), xy_foot);
    smoothedFoot = smoothdata(ttable, 'gaussian', seconds(smoothing_time)).xy_foot;
    d_fp = zeros(size(xy_foot));
    for i = 1:size(smoothedFoot, 2)
        d_fp(:,i) = gradient(smoothedFoot(:,i)) ./ dt;
    end
    vel = vecnorm(d_fp,2,2);
    if debug_plot
        plot(t, smoothedFoot(:,2), "DisplayName", "Smoothed foot position")
        plot(t, vel,  "DisplayName", "Smoothed foot velocity")
    end
    
    mask = and(vel < min_vel, max(smoothedFoot - min_pos,[],2) < 0);
    step_count = 0;
    max_gap = 5;
    miss_counter = inf;
    mask = int32(mask);
    for i = 1:size(t, 1)
        if mask(i)
            if miss_counter > max_gap
                step_count = step_count + 1;
                miss_counter = 0;
                disp(step_count)
            end
            mask(i) = step_count;
        else
            miss_counter = miss_counter+1;
        end
    end

    fp_guess = zeros([step_count, size(xy_foot,2)]);
    times = zeros([step_count, 1]);
    for step = 1:step_count
        fp_guess(step,:) = mean(xy_foot(mask == step,:), 1);
        times(step) = mean(t(mask == step));
    end
    % fp_guess(:,2) = 0;
    mask = mask > 0;
end