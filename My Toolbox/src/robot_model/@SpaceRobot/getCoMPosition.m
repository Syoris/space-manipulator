function comPositions = getCoMPosition(obj)
% getCoMPosition Computes the position of the CoM of all the links in the inertial frame

tTree = obj.forwardKinematics();

comPositions = struct;
n = obj.NumLinks;

% Base
T = tTree.(obj.BaseName) * trvec2tform(obj.Base.CenterOfMass);
comPositions.(obj.BaseName) =  T;

for i = 1:n
    link = obj.Links{i};
    
    % Find transform to parent
    T = tTree.(link.Name) * trvec2tform(link.CenterOfMass);
    comPositions.(link.Name) = T;
end

end