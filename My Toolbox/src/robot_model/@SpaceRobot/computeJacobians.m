function JacM = computeJacobians(obj)
%computeJacobians Compute the Jacobians of all the link CoM to express speed in inertial frame.
%   Output in symbolic form.
%   JacM: struct with link name as fields

    JacM = struct();
    tTree = obj.Ttree_symb;
    
    % Base
    J_b = sym([eye(3), zeros(3), zeros(3, obj.NumActiveJoints);...
               zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)]);
    JacM.(obj.BaseName) = J_b;

    [~, r0_b] = tr2rt(obj.Base.Children{1}.Joint.JointToParentTransform); % Position of first joint in base frame
    [rotM_IB, ~] = tr2rt(tTree.(obj.BaseName)); % Position of first joint in Inertial
    r0_I = rotM_IB*r0_b;

    % Links
    for i =1:obj.NumLinks
        % Build J_i1
        J_1_t1 = zeros(3, 1);

        for k=2:i
            [~, prevLinkLenght] = tr2rt(obj.Links{k}.Joint.JointToParentTransform);
            [prevLinkRotM, ~] = tr2rt(tTree.(obj.Links{k}.Parent.Name)); % Transform of previous joint to Base

            J_1_t1 = J_1_t1 + prevLinkRotM*prevLinkLenght;
        end
        
        [linkRotM, ~] = tr2rt(tTree.(obj.LinkNames{i})); % Transform of link to Base
        J_1_t2 = linkRotM*obj.Links{i}.CenterOfMass.';
        
        J_i1 = -skew(r0_I + J_1_t1 + J_1_t2);

        % J_i2
        J_2_t1 = zeros(3, obj.NumActiveJoints);
        for k=2:i
            [~, prevLinkLenght] = tr2rt(obj.Links{k}.Joint.JointToParentTransform);
            [prevLinkRotM, ~] = tr2rt(tTree.(obj.Links{k}.Parent.Name)); % Transform of previous joint to Base

            res = skew(prevLinkRotM*prevLinkLenght) * obj.getAxisM(k-1);

            J_2_t1 = J_2_t1 + res;
        end
        
        [linkRotM, ~] = tr2rt(tTree.(obj.LinkNames{i})); % Transform of link to Base
        J_2_t2 = skew(linkRotM * obj.Links{i}.CenterOfMass.')*obj.getAxisM(i);
        
        J_i2 = - J_2_t1 - J_2_t2;

        % J_i3
        J_i3 = obj.getAxisM(i);

        J_i = [eye(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
        JacM.(obj.LinkNames{i}) = J_i;
    end
    
    
    f = fields(JacM);
    for i=1:length(f)
        JacM.(f{i}) = simplify(JacM.(f{i}));
    end
    
    obj.JacobsCoM_symb = JacM;
    obj.JacobsCoM_FuncHandle = matlabFunction(struct2array(obj.JacobsCoM_symb), 'Vars', {obj.q_symb});

end