function J = Jacobian(bodyName, sr_info, A, R)
%JACOBIAN Computes jacobian Ji of bodyName
% ti = Ji*q_dot
%
%   The twist ti is expressed in the body frame
%
%   To convert jacobian to inertial frame: J_I = blkdiag(Ri, Ri)*J_i
%       Where Ri is the rot matrix from body i to Inertial frame

%   -sr_info    Struct w/ all fix parameter of SR w/ fields
%
%
%                   - jnt_idx: Idx of each joint generalized coord. -1 if fixed.
%
%                   - N: SR total # of DoF (=n+6)
%
%                   - nk: Manipulator # of bodies
%
%                   - n: Manipulator # of DoF
%
%                   - A: Cell Array w/ twist propagation matrices {Ab, Am}.
%                           - Ab: (6, 6, 1); Base to Anchor point, base frame
%                           - Am: (6, 6, nk); A_i_(i-1), (i-1) frame
%                   - M: Cell Array w/ mass matrices {Mb, Mm}.
%                           - Mb: (6, 6, 1); Base mass mat
%                           - Mm: (6, 6, nk); Body i mass mat, body i frame
%
%                   - P: Cell Array w/ joint rate propagation matrices {Pb, Pm}.
%                            - Pb: (6, 6, 1); Base, base frame
%                            - Pm: (6, 1, nk); Body i, body i frame
%   -A          Twist propaagtion matrices {Ab, Am}. A_i_(i-1) in frame i
%                   -Ab: Manipulator anchor to base, anchor frame
%                   -Am: Array w/ manip matrices
%
%   -R         Rot Matrice: {Rb, Ra}
%                   Rb: from base to inertial frame 
%                   Ra: from anchor to base frame 
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage
    

    [Nkl, Nd, Nbl, Ndb] = DeNOC(sr_info, A, R);      
    
    % --- Jacobians ---   
%     bodyIdx = -1;
    bodyIdx = find(strcmp(sr_info.BodyNames, bodyName));
    bodyIdx = bodyIdx(1);
    assert(bodyIdx>0, 'ERROR: Invalid body name')

    i = bodyIdx;

    Jk_full = Nkl(6*i-5:6*i, :) * Nd;
    Jb = Nbl(6*i-5:6*i, :) * Ndb;
    
    Jk = zeros(6, n);
    
    % Remove fixed joints
    for i=1:nk
        jnt_idx = sr_info.jnt_idx(i);
        if jnt_idx > 0
            Jk(:, jnt_idx) = Jk_full(:, i);
        end
        
    end

    J = [Jb, Jk]; 
end

