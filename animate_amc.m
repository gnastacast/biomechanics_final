 addpath MoCapTools/src/

%%
skeleton = MoCapTools.Skeleton("49.asf");
skeleton.AddMotion("49_04.amc");
skeleton.AddMotion("49_02.amc");

%%
function [G, xyz] = graphSkeleton(skeleton, trial_no, frame)
    G = graph;
    matMap = containers.Map;
    [G, xyz] = graphJoints(G, [], skeleton, skeleton, matMap, trial_no, frame);
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


%%

% figure(1);
% [G, xyz] = graphSkeleton(skeleton, 2, 1);
% % hold on
% p = plot(G, XData=xyz(:,1), YData=xyz(:,2), ZData=xyz(:,3), NodeLabel=repmat("", numel(G.Nodes), 1));
% 
% % plot([-20,20], [0,0], 'k')
% axis equal
% xlim([-20,20])
% zlim([-20,20])
% ylim([-1,40])
% 
% hold on
% 
% for i = 1:5:1500%skeleton.MotionData(2).Frames
%     [G, xyz] = graphSkeleton(skeleton, 2, i);
%     p.XData = xyz(:,1);
%     p.YData = xyz(:,2);
%     p.ZData = xyz(:,3);
%     title(i);
%     refreshdata
%     drawnow
% end
% % updateSkeleton(ax, matMap, skeleton, 2, 1)
% hold on
% axis equal
% hold off;

%%

trial_no = 4;

figure(1);
[G, xyz] = graphSkeleton(skeleton, trial_no, 1);
% p = plot(G, XData=xyz(:,1), YData=xyz(:,2), ZData=xyz(:,3), NodeLabel=repmat("", numel(G.Nodes), 1));
% %%
hold on
plot([-20,20], [0,0], 'k')
p_front = plot(G, XData=xyz(:,1), YData=xyz(:,2), NodeLabel=repmat("", numel(G.Nodes), 1));
p_side = plot(G, XData=xyz(:,3) + 10, YData=xyz(:,2), NodeLabel=repmat("", numel(G.Nodes), 1));
axis equal
xlim([-40,60])
ylim([-1,40])
% for i = 950:5:1900%skeleton.MotionData(trial_no).Frames
i = 1;
while true
% for i = 1:5:skeleton.MotionData(trial_no).Frames
    [G, xyz] = graphSkeleton(skeleton, trial_no, i);
    p_front.XData = xyz(:,1);
    p_front.YData = xyz(:,2);
    p_side.XData = xyz(:,3) + 10;
    p_side.YData = xyz(:,2);
    title(i);
    refreshdata
    drawnow
    i = mod(i + 5, skeleton.MotionData(trial_no).Frames) + 1;
end

axis equal
hold off;
