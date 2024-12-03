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