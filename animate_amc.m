 addpath MoCapTools/src/

run("model5_params.m")

%%
if ~exist("skeleton", "var") || skeleton.Subject ~= 49
    skeleton = MoCapTools.Skeleton("49.asf");
    skeleton.AddMotion("49_04.amc");
    skeleton.AddMotion("49_02.amc");
end
trial_no = 4;

% if ~exist("skeleton", "var") || skeleton.Subject ~= 78
%     skeleton = MoCapTools.Skeleton("78.asf");
%     skeleton.AddMotion("78_12.amc");
% end
% trial_no = 12;


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


%% Plot trial
figure(1);
[G, xyz] = graphSkeleton(skeleton, trial_no, 1);
jointNames = table2array(convertvars(G.Nodes, 'Name', 'string'));
xyzCOG = getCOG(xyz, COMf, jointNames);
% p = plot(G, XData=xyz(:,1), YData=xyz(:,2), ZData=xyz(:,3), NodeLabel=repmat("", numel(G.Nodes), 1));
% %%
hold on
plot([-20,20], [0,0], 'k')
p_COG_front = plot([xyzCOG(1), xyzCOG(1)], [0, xyzCOG(2)], 'ok-', 'MarkerSize', 10);
p_COG_side = plot([xyzCOG(1), xyzCOG(1)], [0, xyzCOG(3)], 'ok-', 'MarkerSize', 10);
p_front = plot(G, XData=xyz(:,1), YData=xyz(:,2), NodeLabel=repmat("", numel(G.Nodes), 1));
p_side = plot(G, XData=xyz(:,3) + 10, YData=xyz(:,2), NodeLabel=repmat("", numel(G.Nodes), 1));
axis equal
xlim([-2,3])
ylim([-.1,2])
% for i = 950:5:1900%skeleton.MotionData(trial_no).Frames
i = 1;
tic
t = [1:skeleton.MotionData(trial_no).Frames]' / 120;
tic
gifFile = "Figures/" + skeleton.Subject + "_" + trial_no + ".gif";
% while true
for i = 1:5:skeleton.MotionData(trial_no).Frames
    % i = find (t > mod(toc, t(end)), 1);
    [G, xyz] = graphSkeleton(skeleton, trial_no, i);
    xyzCOG = getCOG(xyz, COMf, jointNames);
    p_COG_front.XData = [xyzCOG(1), xyzCOG(1)];
    p_COG_front.YData = [0, xyzCOG(2)];
    p_COG_side.XData = [xyzCOG(3), xyzCOG(3)] + .5;
    p_COG_side.YData = [0, xyzCOG(2)];
    p_front.XData = xyz(:,1);
    p_front.YData = xyz(:,2);
    p_side.XData = xyz(:,3) + .5;
    p_side.YData = xyz(:,2);
    title(i);
    % refreshdata
    % drawnow
    exportgraphics(gcf, gifFile, "Append",true);
    i = mod(i + 4, skeleton.MotionData(trial_no).Frames) + 1;
end

hold off;
