 addpath MoCapTools/src/

%%
if ~exist("skeleton", "var")
    skeleton = MoCapTools.Skeleton("49.asf");
    skeleton.AddMotion("49_04.amc");
    skeleton.AddMotion("49_02.amc");
end
trial_no = 4;

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
n_legs = 2;
xyzFP = [xyz_lfoot, xyz_rfoot];
t = [1:n_frames]' * 1/120;

[vels, liftoff, landing, fp] = analyze_data(t, xyz_COG, xyzFP, n_legs, 0.8, [-inf, -inf, 0.1], 0.2, 0.0145, true);

leg_lengths = vecnorm(fp,2,2)


load('polys.mat')

% model parameters
m = 80; % [kg]
g = 9.8; % [m/s^2]
p0 = 0.4; % [m]
l0 = 1 - p0; % [m]
k = 13000; % [N/m]

a0 = 87.9294 * pi/180;  % [rad] : angle of attack during flight

% initial conditions
x0 = liftoff(1,1);
y0 = liftoff(1,2);
dx0= vels(1,1);
dy0= vels(1,2);

model_version = 1;
noise_seed = 0;
noise_gain = 0;

out2 = sim('model5_controlled');

figure()
hold on
plot(xyz_COG(:,1), xyz_COG(:,2), "LineWidth",2)
blanks = [];
for i = 1:2:size(xyz_COG,1)
     plot([xyz_COG(i, 1), xyz_lfoot(i, 1)], ...
          [xyz_COG(i, 2), xyz_lfoot(i,2)], 'Color',[1,.7,.7])
     blanks = [blanks, ""];
end
for i = 1:2:size(xyz_COG,1)
     plot([xyz_COG(i, 1), xyz_rfoot(i, 1)], ...
          [xyz_COG(i, 2), xyz_rfoot(i,2)], 'Color',[.7,.7,1])
     blanks = [blanks, ""];
end
plot(out2.xy.Data(:,1), out2.xy.Data(:,2), "LineWidth",2)
scatter(landing(:,1), landing(:,2))
ends = fp + landing;
scatter(ends(:,1), ends(:,2))
plot([0,10],[0,0], 'k')
for i = 1:size(ends,1)
    plot([landing(i, 1), ends(i, 1)], ...
         [landing(i, 2), ends(i,2)], 'k', "LineWidth",2)
end
axis equal
ylim([0,3])
xlim([-3,3])
title("Results using foot placement controller")
legend(["Original CoM",blanks, "Replica CoM"])