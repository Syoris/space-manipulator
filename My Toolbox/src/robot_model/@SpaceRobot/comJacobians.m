function JacM = comJacobians(obj)
%comJacobians Compute the Jacobian of all the link CoM.
%   JacM: struct with link name as fields

    JacM = struct();
    tTree = obj.forwardKinematics();
    
    % Base
    J_b = [eye(3), zeros(3), zeros(3, obj.NumActiveJoints);...
           zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)];
    JacM.(obj.BaseName) = J_b;


    % Links
    for i =1:obj.NumLinks
        % Build J_i1
        r0_b = tform2trvec(obj.Base.Children{1}.Joint.JointToParentTransform)'; % Position of first joint in base frame
        r0_I = tform2rotm(tTree.(obj.BaseName).Transform)*r0_b;
        J_1_t1 = zeros(3, 1);

        for k=2:i
            prevLinkLenght = obj.Links{k}.Joint.JointToParentTransform;
            prevLinkRotM = tTree.(obj.Links{k}.Parent.Name).Transform;
            J_1_t1 = J_1_t1 + tform2rotm(prevLinkRotM)*tform2trvec(prevLinkLenght)';
        end
        
        J_1_t2 = tform2rotm(tTree.(obj.LinkNames{i}).Transform)*obj.Links{i}.CenterOfMass';
        
        J_i1 = -skew(r0_I + J_1_t1 + J_1_t2);

        % J_i2
        J_2_t1 = zeros(3, obj.NumActiveJoints);
        for k=2:i
            prevLinkLenght = obj.Links{k}.Joint.JointToParentTransform;
            prevLinkRotM = tTree.(obj.Links{k}.Parent.Name).Transform;
            res = skew(tform2rotm(prevLinkRotM)*tform2trvec(prevLinkLenght)')*obj.getAxisM(k-1);
            J_2_t1 = J_2_t1 + res;
        end
        
        J_2_t2 = skew(tform2rotm(tTree.(obj.LinkNames{i}).Transform)*obj.Links{i}.CenterOfMass')*obj.getAxisM(i);
        
        J_i2 = - J_2_t1 - J_2_t2;

        % J_i3
        J_i3 = obj.getAxisM(i);

        J_i = [eye(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
        JacM.(obj.LinkNames{i}) = J_i;
    end

end