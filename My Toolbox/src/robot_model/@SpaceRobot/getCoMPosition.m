function comPositions = getCoMPosition(obj)
% getCoMPosition Computes the position of the CoM of all the bodies in the inertial frame

tTree = obj.Ttree;

comPositions = struct;
n = obj.NumBodies;

% Base
T = tTree.(obj.BaseName) * trvec2tform(obj.Base.CenterOfMass);
comPositions.(obj.BaseName) =  T;

for i = 1:n
    body = obj.Bodies{i};
    
    % Find transform to parent
    T = tTree.(body.Name) * trvec2tform(body.CenterOfMass);
    comPositions.(body.Name) = T;
end

end