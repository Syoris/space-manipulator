%% RFunc_gen.m
function RFunc_gen(sr, savePath)
% RFUNC_GEN Generates function file to get rotation matrix from current
% config.
%
%   File will be named `RFunc_SRNAME.m` and saved at specified savePath.
%
% Input:
%   - sr        SpaceRobot class to generate function for
%
%   - savePath  Where to save file
%
%
% To generate rot matrix function from symbolic expression
% [RbI, Ra, Rm] = RFunc_SRX(q);
% RbI: Rotation matrix from base to inertial frame (3x3)
% Ra: Rotation matrix from anchor to base (6x6)
% Rm: Rotation matrix from i to i-1 (3x nk*3)
% Tee: Hom. transformation matrix for ee (4x4)
%
%

if nargin<2
    savePath = '';
end

%% Create Rot matrix function
nk = sr.NumBodies;
fileName = ['RFunc_', sr.Name];

fullPath = fullfile(savePath, fileName);

[R_bI, ~] = tr2rt(sr.Base.BaseToParentTransform_symb);
Ra_symb = sr.Base.RotM_symb; % Anchor to base RotM
R_array_symb = sym(zeros(3, 3 * nk));
Tee = sr.getTransform('endeffector', 'TargetFrame', 'inertial', 'symbolic', true);

for i = 1:nk
    body = sr.Bodies{i};
    body_idx = body.Id;

    R_i = body.ParentRotM;

    R_array_symb(:, 3 * body_idx - 2:body_idx * 3) = R_i;
end

matlabFunction(R_bI, Ra_symb, R_array_symb, Tee, 'File', fullPath, 'Vars', {sr.q_symb}, 'Outputs', {'Rb', 'Ra', 'Rm', 'Tee'});

codegen(fileName, '-args',{sr.q},'-report', '-o', [fullPath, '_mex']);

end
