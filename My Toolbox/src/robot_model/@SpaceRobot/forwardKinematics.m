function tTree = forwardKinematics(obj)
    % Compute forwardKinematics of the robot. Output an array of the homogenous 
    % transformation matrix of inertial from to link: T_inertial_linkI

    n = obj.NumLinks;
    % Ttree = repmat({eye(4)}, 1, n);
    tTree = struct;
    
    % Base
    baseTransform = obj.Base.BaseToParentTransform; % TODO: Relative to inertial frame
    tTree.(obj.BaseName) = baseTransform;

    for i = 1:n
        link = obj.Links{i};
        
        % Find transform to parent
        TLink2Parent = link.Joint.transformLink2Parent; % Taking into account current config 

        % Find transform to inertial frame
        parentT = tTree.(obj.Links{i}.Parent.Name);
        linkT = parentT * TLink2Parent;

        tTree.(obj.Links{i}.Name) = linkT;

        % if link.ParentId > 0
        %     Ttree{i} = Ttree{link.ParentId} * TLink2Parent;
        % else % If parent is base
        %     Ttree{i} = TLink2Parent;
        % end
    
    end

    obj.Ttree = tTree;
end