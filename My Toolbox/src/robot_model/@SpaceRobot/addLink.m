function addLink(obj, linkIn, parentName)
%addLink Add link to robot config
%   addLink(linkIn, parentName) add linkIn to robot setting parentName as parent link.
%
%   Example:
%
%   See also 
    narginchk(3,3);
    validateattributes(linkIn, {'Link'}, ...
        {'scalar', 'nonempty'},'addBody', 'linkIn');

    % Check if link with same name exists already in robot
    linkId = obj.findLinkIdxByName(linkIn.Name); % Find link id from name
    
    if linkId > -1
        error("Invalid link name. Same name already exists in the robot")
    end

    % Check if parent exists in robot
    pId = obj.findLinkIdxByName(parentName);
    if pId == -1
        error("Invalid parent name")
    end
    
    % TODO: Check joint name collision

    % Update indexes
    obj.NumLinks = obj.NumLinks + 1;
    linkId = obj.NumLinks;

    obj.Links{linkId} = linkIn;
    obj.LinkNames{linkId} = linkIn.Name;
    linkIn.Id = linkId;
    
    if pId > 0
        parent = obj.Links{pId};
    else
        parent = obj.Base;
    end
    
    linkIn.ParentId = pId;
    linkIn.Parent = parent;
    parent.Children{end+1} = linkIn;

    % Add active joints to config and set joints to HomePosition
    if ~strcmp(linkIn.Joint.Type, 'fixed')
        obj.NumActiveJoints = obj.NumActiveJoints + 1;
        linkIn.Joint.Position = linkIn.Joint.HomePosition;
        linkIn.Joint.Q_id = obj.NumActiveJoints;
        obj.JointsConfig(obj.NumActiveJoints) = struct('JointName',linkIn.Joint.Name, ...
            'JointPosition', linkIn.Joint.Position);                 
    end
end