%% Computing of jacobians
load 'SC_2DoF.mat'

% Initial condition
sc.JointsConfig = [pi/4, -pi/2];
sc.BaseConfig = [0.5, 0.5, 0; 0, 0, 0];

%% Jacobian of first link
Jacobians = struct();
tTree = sc.forwardKinematics();
J_S = {{J01, Jm1}; {J02, Jm2}; {J03, Jm3}};

for i =1:sc.NumLinks
    fprintf('\n##### Link %i #####\n', i);
    % Build J_i1
    r0_b = tform2trvec(sc.Base.Children{1}.Joint.JointToParentTransform)'; % Position of first joint in base frame
    r0_I = tform2rotm(tTree.(sc.BaseName).Transform)*r0_b;
    a = zeros(3, 1);

    for k=2:i
        prevLinkLenght = sc.Links{k}.Joint.JointToParentTransform;
        prevLinkRotM = tTree.(sc.Links{k}.Parent.Name).Transform;
        a = a + tform2rotm(prevLinkRotM)*tform2trvec(prevLinkLenght)';
    end
    
    b = tform2rotm(tTree.(sc.LinkNames{i}).Transform)*sc.Links{i}.CenterOfMass';
    
    fprintf('--- J_%i1', i)
    J_i1 = -skew(r0_I + a + b)
    J_i1_S = J_S{i}{1}(4:6, 1:3)

    % J_i2
    a = zeros(3, sc.NumActiveJoints);
    for k=2:i
        prevLinkLenght = sc.Links{k}.Joint.JointToParentTransform;
        prevLinkRotM = tTree.(sc.Links{k}.Parent.Name).Transform;
        a_2 = skew(tform2rotm(prevLinkRotM)*tform2trvec(prevLinkLenght)')*sc.getAxisM(k-1);
        a = a + a_2;
    end
    
    b = skew(tform2rotm(tTree.(sc.LinkNames{i}).Transform)*sc.Links{i}.CenterOfMass')*sc.getAxisM(i);
    
    fprintf('--- J_%i2', i)
    J_i2 = - a - b
    J_i2_S = J_S{i}{2}(4:6, :)

    % J_i3
    fprintf('--- J_%i3', i)
    J_i3 = sc.getAxisM(i)
    J_i3_S = J_S{i}{2}(1:3, :)

    J_i = [eye(3), J_i1, J_i2; zeros(3, 3), eye(3), J_i3];
    Jacobians.(sc.LinkNames{i}) = J_i;
end