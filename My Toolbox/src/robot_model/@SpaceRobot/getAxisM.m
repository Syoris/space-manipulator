%TODO: Test with different robot construction
function AxisM = getAxisM(obj, linkId, frame)
%getAxisM Compute the axis matrix related to the specified link
%   Options: frame. Specified reference frame. Can be either 'inertial' or 'base'. Default: 'inertial'
%
%   Computed as: [R1*Z1, R2*Z2, ..., Ri*Zi, 0]
%   Rj: Rotation matrix from link frame to inertial frame
%   Zj: Rotation axis in link frame

    tTree = obj.Ttree;
    AxisM = zeros(3, obj.NumActiveJoints);
    count = 1; % Active joint count
    narginchk(2, 3);
    if nargin == 2
        frame = 'inertial';
    end

    if ~strcmp(frame, 'base') && ~strcmp(frame, 'inertial')
        error('Wrong frame specified')
    end

    for i=1:obj.NumActiveJoints
        joint = obj.Links{i}.Joint;
        if ~strcmp(joint.Type, 'fixed')        
            if strcmp(frame, 'inertial')
                [rotM, ~] = tr2rt(tTree.(obj.Links{i}.Name));
            else
                [rotM, ~] = tr2rt(obj.getTransform(obj.Links{i}.Name));
            end

            AxisM(:, count) = rotM*joint.Axis';
            count = count + 1;
        end

        if count > linkId 
            break;
        end
    end

end