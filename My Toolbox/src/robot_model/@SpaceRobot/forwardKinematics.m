function tTree = forwardKinematics(obj, varargin)
    % Compute forwardKinematics of the robot. Output an array of the homogenous 
    % transformation matrix of inertial frame to link: T_inertial_linkI
    %       'symbolic'      - Compute tree in symbolic form
    %                           Default: false

    parser = inputParser;
    parser.StructExpand = false;

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
    parser.parse(varargin{:});
    symbolic = parser.Results.symbolic;

    n = obj.NumLinks;
    % Ttree = repmat({eye(4)}, 1, n);
    tTree = struct;
    
    if ~symbolic
        % Base
        baseTransform = obj.Base.BaseToParentTransform;
        tTree.(obj.BaseName) = baseTransform;

        for i = 1:n
            link = obj.Links{i};
            
            % Find transform to parent
            TLink2Parent = link.Joint.transformLink2Parent; % Taking into account current config 

            % Find transform to inertial frame
            parentT = tTree.(obj.Links{i}.Parent.Name);
            linkT = parentT * TLink2Parent;

            tTree.(obj.Links{i}.Name) = linkT;
        end
    
    % Symbolic computation
    else
        % Base
        baseTransform = obj.Base.BaseToParentTransform_symb;
        tTree.(obj.BaseName) = baseTransform;

        for i = 1:n
            link = obj.Links{i};
            
            % Find transform to parent
            TLink2Parent = link.Joint.transformLink2ParentSymb; % Taking into account current config 

            % Find transform to inertial frame
            parentT = tTree.(obj.Links{i}.Parent.Name);
            linkT = parentT * TLink2Parent;

            tTree.(obj.Links{i}.Name) = simplify(linkT);
        end

        obj.Ttree_symb = tTree;
    end
end