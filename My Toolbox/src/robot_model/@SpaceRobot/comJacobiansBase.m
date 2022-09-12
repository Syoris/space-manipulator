function JacM = comJacobiansBase(obj, varargin)
    %comJacobiansBase Compute the Jacobian of all the body CoM in base frame.
    %   JacM: struct with body name as fields
    %   r_i_dot = J_i * q_dot,  r_i_dot: ith Body CoM speed in base frame
    %
    %       'symbolic'      - Compute tree in symbolic form
    %                           Default: false
    %

    parser = inputParser;

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));
    parser.parse(varargin{:});
    symbolic = parser.Results.symbolic;

    JacM = struct();

    % Base
    J_b = [eye(3), zeros(3), zeros(3, obj.NumActiveJoints); ...
        zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)];
    J_b = sym(J_b);
    JacM.(obj.BaseName) = J_b;
    [~, r0_b] = tr2rt(obj.Base.ManipToBaseTransform);

    % Bodies
    for i = 1:obj.NumBodies

        % --- J_i1 ---
        J_1_t1 = zeros(3, 1);

        for k = 1:i - 1
            curBody = obj.Bodies{k};
            nextBody = curBody.Children{1};

            [~, bodyLenght] = tr2rt(obj.getTransform(nextBody.Name, 'TargetFrame', curBody.Name, 'symbolic', false));
            [bodyRotM, ~] = tr2rt(obj.getTransform(curBody.Name, 'TargetFrame', obj.BaseName, 'symbolic', true));

            J_1_t1 = J_1_t1 + bodyRotM * bodyLenght;
        end

        [bodyRotM, ~] = tr2rt(obj.getTransform(obj.Bodies{i}.Name)); % Transform of body to Base

        J_1_t2 = bodyRotM * obj.Bodies{i}.CenterOfMass.';

        J_i1 = -skew(r0_b + J_1_t1 + J_1_t2);

        % --- J_i2 ---
        J_2_t1 = zeros(3, obj.NumActiveJoints);

        for k = 1:i - 1
            curBody = obj.Bodies{k};
            nextBody = curBody.Children{1};

            [~, bodyLenght] = tr2rt(obj.getTransform(nextBody.Name, 'TargetFrame', curBody.Name, 'symbolic', false));
            [bodyRotM, ~] = tr2rt(obj.getTransform(curBody.Name, 'TargetFrame', obj.BaseName, 'symbolic', true));

            res = skew(bodyRotM * bodyLenght) * obj.getAxisM(k, 'base');
            J_2_t1 = J_2_t1 + res;
        end

        [bodyRotM, ~] = tr2rt(obj.getTransform(obj.Bodies{i}.Name)); % Transform of body to Base
        J_2_t2 = skew(bodyRotM * obj.Bodies{i}.CenterOfMass.') * obj.getAxisM(i, 'base');

        J_i2 =- J_2_t1 - J_2_t2;

        % --- J_i3 ---
        J_i3 = obj.getAxisM(i, 'base');

        J_i = [zeros(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
        JacM.(obj.BodyNames{i}) = J_i;
    end

    f = fields(JacM);

    for i = 1:length(f)

        if symbolic
            JacM.(f{i}) = simplify(JacM.(f{i}));
        else
            JacM.(f{i}) = double(subs(JacM.(f{i}), obj.q_symb, obj.q));
        end

    end

    if symbolic
        obj.JacobsCoM_Base_symb = JacM;
    end

end
