function JacM = comJacobiansBase(obj, varargin)
    %comJacobians Compute the Jacobian of all the link CoM in base frame.
    %   JacM: struct with link name as fields
    %   r_i_dot = J_i * q_dot,  r_i_dot: ith Link CoM speed in base frame
    %       'symbolic'      - Compute tree in symbolic form
    %                           Default: false

    parser = inputParser;
    parser.StructExpand = false;

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
    parser.parse(varargin{:});
    symbolic = parser.Results.symbolic;

    JacM = struct();
    
    % Base
    J_b = [eye(3), zeros(3), zeros(3, obj.NumActiveJoints);...
    zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)];
    J_b = sym(J_b);
    JacM.(obj.BaseName) = J_b;
    [~, r0_b] = tr2rt(obj.Base.Children{1}.Joint.JointToParentTransform);

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

        J_1_t2 = linkRotM*obj.Links{i}.CenterOfMass.';
        
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
        J_2_t2 = skew(linkRotM*obj.Links{i}.CenterOfMass.')*obj.getAxisM(i, 'base');
        
        J_i2 = - J_2_t1 - J_2_t2;

        % J_i3
        J_i3 = obj.getAxisM(i, 'base');

        J_i = [zeros(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
        JacM.(obj.LinkNames{i}) = J_i;
    end

    f = fields(JacM);
    for i=1:length(f)
        if symbolic
            JacM.(f{i}) = simplify(JacM.(f{i}));
        else
            JacM.(f{i}) = double(subs(JacM.(f{i}), obj.q_symb, obj.q));
        end
    end
    if symbolic
        obj.CoMJacobsBase_symb = JacM;
    end
end