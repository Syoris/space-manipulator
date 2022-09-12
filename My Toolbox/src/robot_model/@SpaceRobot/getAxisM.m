%TODO: Test with different robot construction
function AxisM = getAxisM(obj, bodyId, frame, varargin)
    %getAxisM Compute the axis matrix related to the specified body
    %   Options: frame. Specified reference frame. Can be either 'inertial' or 'base'. Default: 'inertial'
    %
    %   Computed as: [R1*Z1, R2*Z2, ..., Ri*Zi, 0]
    %   Rj: Rotation matrix from body frame to inertial frame
    %   Zj: Rotation axis in body frame
    %
    %       'TargetFrame'   - To compute matrix wrt to inertial or spacecraft base frame.
    %                         Either `inertial` or `base`.
    %                         Default: 'inertial'
    %
    %       'symbolic'      - Compute matrix in symbolic form
    %                         Default: false
    %
    % Parse Args
    parser = inputParser;

    parser.addParameter('symbolic', false, ...
        @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));

    parser.parse(varargin{:});
    symbolic = parser.Results.symbolic;

    if symbolic
        tTree = obj.Ttree_symb;
    else
        tTree = obj.Ttree;
    end

    AxisM = zeros(3, obj.NumActiveJoints);
    count = 1; % Active joint count

    if nargin == 2
        frame = 'inertial';
    end

    if ~strcmp(frame, 'base') && ~strcmp(frame, 'inertial')
        error('Wrong frame specified')
    end

    for i = 1:obj.NumActiveJoints
        joint = obj.Bodies{i}.Joint;

        if ~strcmp(joint.Type, 'fixed')

            if strcmp(frame, 'inertial')
                [rotM, ~] = tr2rt(tTree.(obj.Bodies{i}.Name));
            else
                [rotM, ~] = tr2rt(obj.getTransform(obj.Bodies{i}.Name, 'symbolic', symbolic));
            end

            ax = rotM * joint.Axis';

            if isa(ax, 'sym') && count == 1
                AxisM = sym(zeros(3, obj.NumActiveJoints));
            end

            AxisM(:, count) = ax;
            count = count + 1;
        end

        if count > bodyId
            break;
        end

    end

end
