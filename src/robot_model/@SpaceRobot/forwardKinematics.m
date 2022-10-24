function tTree = forwardKinematics(obj, varargin)
    % Compute forwardKinematics of the robot. Output an array of the homogenous
    % transformation matrix of inertial frame to body: T_inertial_bodyI
    %       'symbolic'      - Compute tree in symbolic form
    %                           Default: false
    parser = inputParser;
    parser.StructExpand = false;

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));
    parser.parse(varargin{:});
    symbolic = parser.Results.symbolic;

    msg = sprintf('Computing Kin Tree. Symbolic form: %s', string(symbolic));
    obj.logger(msg, 'info');

    n = obj.NumBodies;
    tTree = struct;

    % Base
    if ~symbolic
        baseTransform = obj.Base.BaseToParentTransform; % Base position in Inertial Frame
    else
        baseTransform = obj.Base.BaseToParentTransform_symb;
    end

    tTree.(obj.BaseName) = baseTransform;

    for i = 1:n
        msg = sprintf('Kinematic of body %i', i);
        obj.logger(msg, 'debug');

        body = obj.Bodies{i};

        % Find transform to parent
        if ~symbolic
            TBody2Parent = body.Joint.transformBody2Parent; % Taking into account current config
        else
            TBody2Parent = body.Joint.transformBody2ParentSymb; % Taking into account current config
        end

        % Find transform to inertial frame
        parent = obj.Bodies{i}.Parent;
        parentT = tTree.(parent.Name);

        % Add Manip to Base transform for 1st body
        if parent.Id == 0
            parentT = parentT * parent.ManipToBaseTransform;
        end

        bodyT = parentT * TBody2Parent;

        tTree.(obj.Bodies{i}.Name) = bodyT;

    end

    if symbolic
        %         % Simplify Results
        %         f = fields(tTree);
        %         for i=1:length(f)
        %             mat = tTree.(f{i});
        %
        %             for j=1:size(mat, 1)
        %                 for k=1:size(mat, 1)
        %                     msg = sprintf('Simplifying %s ... (%i, %i)', f{i}, j, k);
        %                     obj.logger(msg, 'debug');
        %                     mat(j, k) = simplify(mat(j, k), 'IgnoreAnalyticConstraints',true,'Seconds',10);
        %                 end
        %             end
        %             tTree.(f{i}) = mat;
        %         end

        obj.Ttree_symb = tTree;
        obj.tTreeFuncHandle = matlabFunction(struct2array(obj.Ttree_symb), 'Vars', {obj.q_symb});
    end

end
