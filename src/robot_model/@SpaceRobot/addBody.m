function addBody(obj, bodyIn, parentName)
    %addBody Add body to robot config
    %   addBody(bodyIn, parentName) add bodyIn to robot setting parentName as parent body.
    %
    %   Example:
    %
    %   See also
    narginchk(3, 3);
    validateattributes(bodyIn, {'Body'}, ...
        {'scalar', 'nonempty'}, 'addBody', 'bodyIn');

    % Check if body with same name exists already in robot
    bodyId = obj.findBodyIdxByName(bodyIn.Name); % Find body id from name

    if bodyId > -1
        error("Invalid body name. Same name already exists in the robot")
    end

    % Check if parent exists in robot
    pId = obj.findBodyIdxByName(parentName);

    if pId == -1
        error("Invalid parent name")
    end

    % TODO: Check joint name collision

    % Update indexes
    bodyId = obj.NumBodies + 1;

    obj.Bodies{bodyId} = bodyIn;
    obj.BodyNames{bodyId} = bodyIn.Name;
    bodyIn.Id = bodyId;

    if pId > 0
        parent = obj.Bodies{pId};
    else
        parent = obj.Base;
    end

    bodyIn.ParentId = pId;
    bodyIn.Parent = parent;
    bodyIn.Joint.ParentBody = parent;
    parent.Children{end + 1} = bodyIn;

    % Add active joints to config and set joints to HomePosition
    if ~strcmp(bodyIn.Joint.Type, 'fixed')
        obj.NumActiveJoints = obj.NumActiveJoints + 1;
        bodyIn.Joint.Position = bodyIn.Joint.HomePosition;
        bodyIn.Joint.Q_id = obj.NumActiveJoints;

        % Add to symbolic vector
        qm_symb = sym('qm', [obj.NumActiveJoints, 1], 'real');
        qm_dot_symb = sym('qm_dot', [obj.NumActiveJoints, 1], 'real');

        obj.q_symb = [obj.q_symb(1:6); qm_symb];
        obj.q_dot_symb = [obj.q_dot_symb(1:6); qm_dot_symb];

        bodyIn.Joint.SymbVar = qm_symb(end);
    end

end
