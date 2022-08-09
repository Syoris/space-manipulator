function JacM = comJacobiansBase(obj)
%comJacobians Compute the Jacobian of all the link CoM in base frame.
%   JacM: struct with link name as fields
%   r_i_dot = J_i * q_dot,  r_i_dot: ith Link CoM speed in base frame

    JacM = struct();
    tTree = obj.Ttree;
    
    % Base
    J_b = [eye(3), zeros(3), zeros(3, obj.NumActiveJoints);...
           zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)];
    JacM.(obj.BaseName) = J_b;

    r0_b = tform2trvec(obj.Base.Children{1}.Joint.JointToParentTransform)'; % Position of first joint in base frame

    % Links
    for i =1:obj.NumLinks
        % Build J_i1
        J_1_t1 = zeros(3, 1);

        for k=2:i
            [~, prevLinkLenght] = tr2rt(obj.Links{k}.Joint.JointToParentTransform);
            [prevLinkRotM, ~] = tr2rt(obj.getTransform(obj.Links{k-1}.Name)); % Transform of previous joint to Base

            J_1_t1 = J_1_t1 + prevLinkRotM*prevLinkLenght;
        end
        
        [linkRotM, ~] = tr2rt(obj.getTransform(obj.Links{i}.Name)); % Transform of link to Base

        J_1_t2 = linkRotM*obj.Links{i}.CenterOfMass';
        
        J_i1 = -skew(r0_b + J_1_t1 + J_1_t2);

        % J_i2
        J_2_t1 = zeros(3, 1);

        for k=2:i
            [~, prevLinkLenght] = tr2rt(obj.Links{k}.Joint.JointToParentTransform);
            [prevLinkRotM, ~] = tr2rt(obj.getTransform(obj.Links{k-1}.Name)); % Transform of previous joint to Base

            res = skew(prevLinkRotM*prevLinkLenght)*obj.getAxisM(k-1, 'base');
            J_2_t1 = J_2_t1 + res;
        end
        
        [linkRotM, ~] = tr2rt(obj.getTransform(obj.Links{i}.Name)); % Transform of link to Base
        J_2_t2 = skew(linkRotM*obj.Links{i}.CenterOfMass')*obj.getAxisM(i, 'base');
        
        J_i2 = - J_2_t1 - J_2_t2;

        % J_i3
        J_i3 = obj.getAxisM(i, 'base');

        J_i = [zeros(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
        JacM.(obj.LinkNames{i}) = J_i;
    end

end