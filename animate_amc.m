 addpath MoCapTools/src/

%%
if ~exist("skeleton", "var")
    skeleton = MoCapTools.Skeleton("49.asf");
    skeleton.AddMotion("49_04.amc");
    skeleton.AddMotion("49_02.amc");
end

%%
function [G, xyz] = graphSkeleton(skeleton, trial_no, frame)
    G = graph;
    matMap = containers.Map;
    [G, xyz] = graphJoints(G, [], skeleton, skeleton, matMap, trial_no, frame);
    % Convert to meters
    xyz = xyz .* 0.0254 / skeleton.Length;
    return
end

function [G,xyz] = graphJoints(G, xyz, joint, skeleton, matMap, trial_no, frame)
    name = joint.Name;
    mat = eye(4);
    if skeleton == joint
        name = "root";
        parent_mat = eye(4);
    else
        parent_mat = matMap(joint.Parent);
        joint_vec = joint.Direction * joint.Length;
    end
    
    data = skeleton.MotionData(trial_no).Data(name);
    if name == "root"
        rot_vec = data(frame,4:6);
        rot_mat = eul2rotm(-rot_vec / 180 * pi, "XYZ")';
        mat(1:3,1:3) = rot_mat;
        mat(1:3, 4) = data(frame,1:3)';

    else
        rot_vec = zeros(1,3);
        if joint.DoF ~= ""
            axisMap = containers.Map(["rx", "ry", "rz"], [1, 2, 3]);
            for i = 1:numel(joint.DoF) 
                rot_vec(axisMap(joint.DoF(i))) = data(frame, i);
            end
        end
        rot_mat = eul2rotm(-rot_vec / 180 * pi, "XYZ")';
        C = eul2rotm(-joint.Axis / 180 * pi, "XYZ")';
        mat(1:3,1:3) = C * rot_mat / C;
        mat(1:3, 4) = mat(1:3,1:3) * joint_vec';
        mat = parent_mat * mat;
    end
    
    matMap(name) =  mat;
    xyz = [xyz; mat(1:3,4)'];
    

    if isempty(joint.Children)
        return
    end

    for child = joint.Children
        G = addedge(G, name, child);
        [G, xyz] = graphJoints(G, xyz, skeleton.Joints(child), skeleton, matMap, trial_no, frame);
    end
end

%% Calculate center of gravity

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

function xyzCOG = getCOG(xyz, COMtable, jointNames)
    xyzCOG = zeros(1,3);
    for i = 1:height(COMtable)
        prefixes = "";
        if COMtable{i, 2}
            prefixes = ["r", "l"];
        end
        for prefix = prefixes
            total = zeros(1,3);
            for joint = COMtable{i, 4}{1}'
                jointName = prefix + joint{1};
                jointIDX = jointNames == jointName;
                assert(sum(jointIDX) == 1)
                total = total + xyz(jointIDX,:) .* joint{2};
                disp(jointName + " " + joint{2})
            end
            xyzCOG = xyzCOG + total .* (COMtable{i, 3} / 100);
        end
    end
end


%% Plot trial

trial_no = 2;
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
xlim([-2,2])
ylim([-.1,2])
% for i = 950:5:1900%skeleton.MotionData(trial_no).Frames
i = 1;
while true
% for i = 1:5:skeleton.MotionData(trial_no).Frames
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
    refreshdata
    drawnow
    i = mod(i + 5, skeleton.MotionData(trial_no).Frames) + 1;
end

hold off;
