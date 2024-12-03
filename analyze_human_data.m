 addpath MoCapTools/src/

%%
if ~exist("skeleton", "var")
    skeleton = MoCapTools.Skeleton("49.asf");
    skeleton.AddMotion("49_04.amc");
    skeleton.AddMotion("49_02.amc");
end
%%

% From S. Plagenhoef 'Anatomical Data for Analyzing Human Motion' 1983

tableCols = {'Name', 'isLRPair', 'Percentage', 'JointWeights'};

% Percentages in the paper do not add up to 100 so the difference was
% subtracted from AbdomenAndPelvis

COMm = cell2table({
	'Hand', true, 0.65, {'radius', 0.468; 'fingers', 0.532};
	'Forearm', true, 1.87, {'humerus', 0.43; 'radius', 0.57} ;
	'UpperArm', true, 3.25, {'clavicle', 0.436; 'humerus', 0.564};
	'Foot', true, 1.43, {'tibia', 0.5; 'toes', 0.5};
	'Shank', true, 4.75, {'femur', 0.434; 'tibia', 0.566};
	'Thigh', true, 10.5, {'hipjoint', 0.433; 'femur', 0.567};
	'HeadAndNeck', false, 8.26, {'lowerneck', 0.55; 'head', 0.45};
	'Thorax', false, 20.1, {'upperback', 0.567; 'lowerneck', 0.433};
	'AbdomenAndPelvis', false, 26.72 + 0.02, {'lhipjoint', 0.2225; 'rhipjoint', 0.2225; 'upperback', 0.555};
}, 'VariableNames', tableCols);


COMf = cell2table({
	'Hand', true, 0.5, {'radius', 0.468; 'fingers', 0.532};
	'Forearm', true, 1.57, {'humerus', 0.434; 'radius', 0.566} ;
	'UpperArm', true, 2.9 {'clavicle', 0.458; 'humerus', 0.542};
	'Foot', true, 1.33 {'tibia', 0.5; 'toes', 0.5};
	'Shank', true, 5.35 {'femur', 0.419; 'tibia', 0.581};
	'Thigh', true, 11.75 {'hipjoint', 0.428; 'femur', 0.572};
	'HeadAndNeck', false, 8.2 {'lowerneck' 0.55; 'head', 0.45};
	'Thorax', false, 17.02 {'upperback', 0.563; 'lowerneck', 0.437};
	'AbdomenAndPelvis', false, 28.2 - 0.22, {'lhipjoint', 0.195; 'rhipjoint', 0.195; 'upperback', 0.61};
}, 'VariableNames', tableCols);


trial_no = 4;
[G, xyz] = graphSkeleton(skeleton, trial_no, 1);
jointNames = table2array(convertvars(G.Nodes, 'Name', 'string'));
n_frames = skeleton.MotionData(trial_no).Frames;

xyz_lfoot = zeros(n_frames, 3);
xyz_rfoot = zeros(n_frames, 3);
xyz_COG = zeros(n_frames, 3);
t = [1:n_frames]' * 1/120;

drawing_data = zeros(n_frames, 31 * 3 + 1);

for i = 1:skeleton.MotionData(trial_no).Frames
    [G, xyz] = graphSkeleton(skeleton, trial_no, i);
    jointIDX = jointNames == 'ltoes';
    xyz = xyz(:, [3 2 1]);
    xyz_lfoot(i, :) = xyz(jointIDX, :);
    jointIDX = jointNames == 'rtoes';
    xyz_rfoot(i, :) = xyz(jointIDX, :);
    xyz_COG(i, :) = getCOG(xyz, COMf, jointNames);
    drawing_data(i, :) = [t(i), reshape(xyz, 1, [])];
end

%%
hold on;
plot(t(:,1), xy_vec_l(:,2));
plot(t(:,1), xy_vec_r(:,2));
axis equal

% figure;
% xyFP_vec = out.xyFP.Data;
% xy_vec = out.xy.Data;
% 
% noise = (rand(size(xy_vec)) - 0.5) * 2 * .01 .* [1 1];
% xy_vec = xy_vec + noise;
% noise = (rand(size(xy_vec)) - 0.5) * 2 * .01 .* [1 1];
% xyFP_vec = xyFP_vec + noise;
% t = out.xyFP.Time;


function fp_guess = find_foot_placements(xy_foot, t, min_vel, min_pos)
    dt = gradient(t);
    smooth_window = round(0.5 / mean(dt));
    smoothedFoot = smoothdata(xy_foot, 1, 'sgolay', smooth_window);
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

disp("L")
fp_guess_l = find_foot_placements(xy_vec_l, t, 1, [-inf, -inf, 0.5]);
[mask_l, times_l] = find_contact(xy_vec_l, fp_guess_l, t, 0.0145);
disp("R")
fp_guess_r = find_foot_placements(xy_vec_r, t, 1, [-inf, -inf, 0.5]);
[mask_r, times_r] = find_contact(xy_vec_r, fp_guess_r, t, 0.0145);

[times, order] = sort([times_l; times_r]);
mask = or(mask_l, mask_r);
fp_guess = [fp_guess_l; fp_guess_r];
fp_guess = fp_guess(order,:);


% % disp(find_foot_placements(xyFP_vec, out.xyFP.Time))
% % plot(out.in_stance.Time, out.in_stance.Data);
% % plot(t(:), sqrt(dy_fp.^2 + dx_fp.^2) / 10);
% % plot(t(:), smoothedXYFPvec(:,2));
% [mask, times] = find_contact(xyFP_vec, t);
hold on
scatter(t(not(mask)), xyz_COG(not(mask),2))
scatter(t(not(mask)), xyz_COG(not(mask),1))
scatter(times, ones(size(times)))

% 

hold on
velocities = [];
positions = [];
foot_placement = [];
scatter(times, ones(size(times)))
for i = 1:length(times)
    if i == 0
        t_start = times(1);
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
    py = polyfit(sub_time, xyz_COG(sub_mask, 2), 2);
    px = polyfit(sub_time, xyz_COG(sub_mask, 1), 1);
    plot(sub_time, polyval(py, sub_time));
    plot(sub_time, polyval(px, sub_time));
    
    % plot(sub_time, sub_time .^ 2 * p(1) + sub_time .* p(2) + p(3))
    % plot(sub_time, 2 * sub_time * p(1) + p(2))
    yvel =  2 * sub_time(1) * py(1) + py(2);
    velocities = [velocities; px(1), yvel];
    disp(velocities(end,1))
    positions = [positions; [polyval(px, t_start), polyval(py, t_start)]];
    if i < length(times)
        foot_xy = [polyval(px, t_end) - fp_guess(i+1,1), ...
                   polyval(py, t_end), fp_guess(i+1,2)];
        foot_placement = [foot_placement; foot_xy];
    end

end

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
x0 = positions(1,1);
y0 = positions(1,2);
dx0= velocities(1,1);
dy0= velocities(1,2);

model_version = 1;
noise_seed = 0;
noise_gain = 0;

sim('model5_controlled');

axis equal