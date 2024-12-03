
% model parameters
m = 80; % [kg]
g = 9.81; % [m/s^2]
p0 = 0.4; % [m]
l0 = 1 - p0; % [m]
k = 20000; % [N/m]

a0 = 68 * pi/180;  % [rad] : angle of attack during flight

% initial conditions
x0 = 0;
dx0 = 5;
y0 = 1;
dy0 = 0; % start at apex in flight

model_version = 1;
noise_seed = 0;
noise_gain = 0;

load('polys.mat')
out = sim('model5_2f.slx');

%%

function fp_guess = find_foot_placements(xy_foot, t, min_vel, min_pos, smoothing_time)
    dt = gradient(t);
    ttable = timetable(seconds(t), xy_foot);
    smoothedFoot = smoothdata(ttable, 'gaussian', seconds(smoothing_time)).xy_foot;
    dx_fp = gradient(smoothedFoot(:,1)) ./ dt;
    dy_fp = gradient(smoothedFoot(:,2)) ./ dt;
    vel = sqrt(dy_fp(:,1).^2 + dx_fp.^2);

    fp_guess = [];
    accumulator = [];
    for i = 1:10:length(t)
        if vel(i) < .8 && any(xy_foot(i,:) < min_pos)
           accumulator = [accumulator; xy_foot(i,:)];
        elseif ~isempty(accumulator)
            next = mean(accumulator,1);
            if isempty(fp_guess) || norm(fp_guess(end,:) - next) > min_vel
                if length(accumulator) > 2
                    fp_guess = [fp_guess; next];
                end
            end
            accumulator = [];
        end
    end
end

function [mask, times] = find_contact(xy_foot, fp_guess, t, max_distance)
    mask = zeros(size(t));
    times = zeros(length(fp_guess), 1);
    for i = 1:length(fp_guess)
        pt = fp_guess(i,:);
        next_mask = vecnorm(xy_foot - pt, 2, 2) < max_distance;
        times(i) = mean(t(next_mask));
        mask = or(mask, next_mask);
    end
end


figure;
xyFP_vec = out.xyFP.Data;
t = out.xy.Time;
xy_vec = out.xy.Data;
noise_mag = 0.00;
noise = rand(size(xy_vec)) * noise_mag .* [1 1];
xy_vec = xy_vec + noise;
xyFP_vec = xyFP_vec + noise;


% mask = gradient(t) < 0.0001;
% t(mask) = [];
% xyFP_vec(mask,:) = [];
% xy_vec(mask,:) = [];

hold on;
plot(t(:,1), xy_vec(:,2));
axis equal

fp_guess = find_foot_placements(xyFP_vec, t, 0.02, [-inf,0.05], 0.01);
[mask, times] = find_contact(xyFP_vec, fp_guess, t, 0.02);


% % disp(find_foot_placements(xyFP_vec, out.xyFP.Time))
% % plot(out.in_stance.Time, out.in_stance.Data);
% % plot(t(:), sqrt(dy_fp.^2 + dx_fp.^2) / 10);
% % plot(t(:), smoothedXYFPvec(:,2));
% [mask, times] = find_contact(xyFP_vec, t);
hold on
scatter(t(not(mask)), xy_vec(not(mask),2))
% scatter(t(not(mask)), xy_vec(not(mask),1))
scatter(times, ones(size(times)))

% 

hold on
velocities = [];
liftoff = [];
landing = [];
foot_placement = [];
scatter(times, ones(size(times)))
for i = 0:length(times)
    if i == 0
        t_start = t(1);
    else
        t_start = times(i);
    end
    if i < length(times)
        t_end = times(i + 1);
    else
        t_end = t(end);
    end
    sub_mask = and(not(mask), and(t > t_start, t < t_end));
    sub_time = t(sub_mask);
    t_start = sub_time(1);
    t_end = sub_time(end);
    if sum(sub_mask) < 3
        continue
    end
    py = polyfit(sub_time, xy_vec(sub_mask, 2), 2);
    px = polyfit(sub_time, xy_vec(sub_mask, 1), 1);
    plot(sub_time, polyval(py, sub_time));
    
    yvel =  2 * sub_time(1) * py(1) + py(2);
    velocities = [velocities; px(1), yvel];
    liftoff = [liftoff; [polyval(px, t_start), polyval(py, t_start)]];
    if i < length(times)
        disp(fp_guess(i+1,:))
        foot_xy = [fp_guess(i+1,1) - polyval(px, t_end), ...
                   fp_guess(i+1,2) - polyval(py, t_end)];
        landing = [landing; [polyval(px, t_end), polyval(py, t_end)]];
        foot_placement = [foot_placement; foot_xy];
    end

end
% 
% figure()
% scatter(foot_placement(:,1) + landings(:,1), foot_placement(:,2) + landings(:,2))

axis equal
%%

load('polys.mat')

% model parameters
m = 62; % [kg]
g = 9.8; % [m/s^2]
p0 = 0.4; % [m]
l0 = 1 - p0; % [m]
k = 20000; % [N/m]

a0 = 87.9294 * pi/180;  % [rad] : angle of attack during flight

% initial conditions
x0 = liftoff(1,1);
y0 = liftoff(1,2);
dx0= velocities(1,1);
dy0= velocities(1,2);

model_version = 1;
noise_seed = 0;
noise_gain = 0;

sim('model5_controlled');

axis equal