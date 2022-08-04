%TODO: Test with different configurations
function AxisM = getAxisM(obj, linkId)
%getAxisM Compute the axis matrix related to the specified link
%   Computed as: [R1*Z1, R2*Z2, ..., Ri*Zi, 0]
%   Rj: Rotation matrix from link frame to inertial frame
%   Zj: Rotation axis in link frame

    tTree = obj.forwardKinematics();
    AxisM = zeros(3, obj.NumActiveJoints);
    count = 1; % Active joint count

    for i=1:obj.NumActiveJoints
        joint = obj.Links{i}.Joint;
        if ~strcmp(joint.Type, 'fixed')        
            AxisM(:, count) = tform2rotm(tTree.(obj.Links{i}.Name).Transform)*joint.Axis';
            count = count + 1;
        end

        if count > linkId break; 
    end

end