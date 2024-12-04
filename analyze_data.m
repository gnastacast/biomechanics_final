function [vels, liftoff, landing, fp] = analyze_data(t, xy, xyFP, ....
    n_feet, min_vel, min_pos, smoothing_time, max_distance, debug_plot)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    if debug_plot
        plot(t, xy(:,2));
        hold on
    end
    
    mask = zeros(size(t));
    times = [];
    fp_guess = [];
    for leg=1:n_feet
        leg_cols = size(xyFP,2) / n_feet;
        xyFP_leg = xyFP(:, (leg-1) * leg_cols + 1 : leg * leg_cols);
        next_fp_guess = find_foot_placements(xyFP_leg, t, min_vel, min_pos, smoothing_time, debug_plot);
        [next_mask, next_times] = find_contact(xyFP_leg, next_fp_guess, t, max_distance, debug_plot);
        mask = or(mask, next_mask);
        if size(next_times, 1) ~= size(next_fp_guess, 1)
            assert(size(next_times, 1) == size(next_fp_guess, 1));
        end
        times = [times; next_times];
        fp_guess = [fp_guess; next_fp_guess];
        disp(next_times);
        disp(next_fp_guess);
    end

    [times, order] = sort(times, 1);
    fp_guess = fp_guess(order,:);
    
    if debug_plot
        scatter(t(not(mask)), xy(not(mask),2))
        scatter(times, ones(size(times)))
        scatter(times, fp_guess(:,2))
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
            plot(sub_time, polyval(py, sub_time));
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
        hold off
        axis equal
    end

end

function fp_guess = find_foot_placements(xy_foot, t, min_vel, min_pos, smoothing_time, debug_plot)
    dt = gradient(t);
    ttable = timetable(seconds(t), xy_foot);
    smoothedFoot = smoothdata(ttable, 'gaussian', seconds(smoothing_time)).xy_foot;
    dx_fp = gradient(smoothedFoot(:,1)) ./ dt;
    dy_fp = gradient(smoothedFoot(:,2)) ./ dt;

    vel = sqrt(dy_fp(:,1).^2 + dx_fp.^2);
    if debug_plot
        plot(t(vel<10), vel(vel<10) / median(vel(vel<100)) / 2)
    end
    
    fp_guess = [];
    accumulator = [];
    n_frames =  size(t,1);
    for i = 1:1:n_frames+1
        if i <= n_frames && vel(i) < min_vel && any(xy_foot(i,:) < min_pos)
           accumulator = [accumulator; xy_foot(i,:)];
        elseif ~isempty(accumulator)
            next = mean(accumulator,1);
            if isempty(fp_guess) || norm(fp_guess(end,:) - next) > min_vel
                if size(accumulator, 1) > 2
                    fp_guess = [fp_guess; next];
                end
            end
            accumulator = [];
        end
    end
end

function [mask, times] = find_contact(xy_foot, fp_guess, t, max_distance, debug_plot)
    mask = zeros(size(t));
    n_times = size(fp_guess,1);
    times = zeros(n_times, 1);
    for i = 1:n_times
        pt = fp_guess(i,:);
        next_mask = vecnorm(xy_foot - pt, 2, 2) < max_distance;
        times(i) = mean(t(next_mask));
        mask = or(mask, next_mask);
    end
end