%% RFunc_gen.m
% To generate rot matrix function from symbolic expression
% [RbI, Ra, Rm] = RFunc_SRX(q);
% RbI: Rotation matrix from base to inertial frame (3x3)
% Ra: Rotation matrix from anchor to base (6x6)
% Rm: Rotation matrix from i to i-1 (3x nk*3)

%% Create Rot matrix function
nk = sr.NumBodies;
fileName = ['RFunc_', sr.Name];

[R_bI, ~] = tr2rt(sr.Base.BaseToParentTransform_symb);
Ra_symb = sr.Base.RotM_symb; % Anchor to base RotM
R_array_symb = sym(zeros(3, 3 * nk));

for i = 1:nk
    body = sr.Bodies{i};
    body_idx = body.Id;

    R_i = body.ParentRotM;

    R_array_symb(:, 3 * body_idx - 2:body_idx * 3) = R_i;
end

matlabFunction(R_bI, Ra_symb, R_array_symb, 'File', fileName, 'Vars', {sr.q_symb}, 'Outputs', {'Rb', 'Ra', 'Rm'});
