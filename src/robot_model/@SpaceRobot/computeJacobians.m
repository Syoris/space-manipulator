function JacM = computeJacobians(obj, varargin)
    %comJacobiansBase Compute the Jacobian of all the body CoM in the specified TargetFrame.
    %   JacM: struct with body name as fields
    %   [r_i_dot, w_i] = JacM.bodyName_i * q_dot
    %       r_i_dot:    ith Body CoM speed in base frame
    %       w_i:        ith Body angular speed in target frame
    %
    %       'TargetFrame'   - To compute Jacobians wrt to inertial or spacecraft base frame.
    %                         Either `inertial` or `base`.
    %                         Default: 'inertial'
    %
    %       'symbolic'      - Compute jacobians in symbolic form
    %                         Default: false
    %

    % Parse Args
    parser = inputParser;

    parser.addParameter('TargetFrame', 'inertial', ...
        @(x)any(validatestring(x, {'inertial', 'base'})));

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));

    parser.parse(varargin{:});
    targetFrame = parser.Results.TargetFrame;
    symbolic = parser.Results.symbolic;

    msg = sprintf('Computing Symbolic CoM Jacobians. Symbolic: %s. TargetFrame: %s', string(symbolic), targetFrame);
    obj.logger(msg, 'info');

    %
    JacM = struct();
    tTree = obj.Ttree_symb;

    % Base
    J_b = [eye(3), zeros(3), zeros(3, obj.NumActiveJoints); ...
        zeros(3, 3), eye(3), zeros(3, obj.NumActiveJoints)];

    if symbolic
        J_b = sym(J_b);
    end

    JacM.(obj.BaseName) = J_b;

    % Manip position
    [~, r0_b] = tr2rt(obj.Base.ManipToBaseTransform); % Position of first joint in base frame

    switch targetFrame
        case 'inertial'
            [rotM_t_B, ~] = tr2rt(tTree.(obj.BaseName)); % Rotation matrix of base frame to inertial frame
        case 'base'
            rotM_t_B = eye(3);
    end

    r0_t = rotM_t_B * r0_b; % Position of first joint in target frame

    % Bodies
    for i = 1:obj.NumBodies
        msg = sprintf('Computing for body %i', i);
        obj.logger(msg, 'debug');

        % --- J_i1 ---
        % J_i1_t1
        J_1_t1 = zeros(3, 1);

        for k = 1:i - 1
            curBody = obj.Bodies{k};
            nextBody = curBody.Children{1};

            [~, bodyLenght] = tr2rt(obj.getTransform(nextBody.Name, 'TargetFrame', curBody.Name, 'symbolic', false));

            switch targetFrame
                case 'inertial'
                    [bodyRotM, ~] = tr2rt(tTree.(curBody.Name));
                case 'base'
                    [bodyRotM, ~] = tr2rt(obj.getTransform(curBody.Name, 'TargetFrame', obj.BaseName, 'symbolic', symbolic));
            end

            J_1_t1 = J_1_t1 + bodyRotM * bodyLenght;
        end

        % J_i1_t2
        switch targetFrame
            case 'inertial'
                [bodyRotM, ~] = tr2rt(tTree.(obj.BodyNames{i})); % Transform of body to inertial frame
            case 'base'
                [bodyRotM, ~] = tr2rt(obj.getTransform(obj.Bodies{i}.Name, 'symbolic', symbolic)); % Transform of body to Base frame
        end

        J_1_t2 = bodyRotM * obj.Bodies{i}.CenterOfMass.';

        J_i1 = -skew(r0_t + J_1_t1 + J_1_t2);

        % --- J_i2 ---
        J_2_t1 = zeros(3, obj.NumActiveJoints);

        for k = 1:i - 1
            curBody = obj.Bodies{k};
            nextBody = curBody.Children{1};

            [~, bodyLenght] = tr2rt(obj.getTransform(nextBody.Name, 'TargetFrame', curBody.Name, 'symbolic', false));

            switch targetFrame
                case 'inertial'
                    [bodyRotM, ~] = tr2rt(tTree.(curBody.Name));
                    E = obj.getAxisM(k);
                case 'base'
                    [bodyRotM, ~] = tr2rt(obj.getTransform(curBody.Name, 'TargetFrame', obj.BaseName, 'symbolic', symbolic));
                    E = obj.getAxisM(k, 'base');
            end

            res = skew(bodyRotM * bodyLenght) * E;
            J_2_t1 = J_2_t1 + res;
        end

        switch targetFrame
            case 'inertial'
                [bodyRotM, ~] = tr2rt(tTree.(obj.BodyNames{i})); % Transform of body to inertial frame
                E = obj.getAxisM(i);
            case 'base'
                [bodyRotM, ~] = tr2rt(obj.getTransform(obj.Bodies{i}.Name, 'symbolic', symbolic)); % Transform of body to Base frame
                E = obj.getAxisM(i, 'base');
        end

        J_2_t2 = skew(bodyRotM * obj.Bodies{i}.CenterOfMass.') * E;

        J_i2 =- J_2_t1 - J_2_t2;

        % --- J_i ---
        switch targetFrame
            case 'inertial'
                J_i0 = eye(3);
            case 'base'
                J_i0 = zeros(3);
        end

        J_i = [J_i0, J_i1, J_i2; zeros(3, 3), eye(3), E];
        JacM.(obj.BodyNames{i}) = J_i;
    end

    if symbolic
        %         % Simplify Results
        %         f = fields(JacM);
        %         for i=1:length(f)
        %             mat = JacM.(f{i});
        %
        %             for j=1:size(mat, 1)
        %                 for k=1:size(mat, 1)
        %                     msg = sprintf('Simplifying %s ... (%i, %i)', f{i}, j, k);
        %                     obj.logger(msg, 'debug');
        %                     mat(j, k) = simplify(mat(j, k), 'IgnoreAnalyticConstraints',true,'Seconds',10);
        %                 end
        %             end
        %             JacM.(f{i}) = mat;
        %         end

        % Update SpaceRobot Parameters
        switch targetFrame
            case 'inertial'
                obj.JacobsCoM_symb = JacM;
                obj.JacobsCoM_FuncHandle = matlabFunction(struct2array(obj.JacobsCoM_symb), 'Vars', {obj.q_symb});

            case 'base'
                obj.JacobsCoM_Base_symb = JacM;

        end

    end

end
