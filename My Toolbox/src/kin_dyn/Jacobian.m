function J = Jacobian(bodyName, sr_info, A, R)
%JACOBIAN Computes jacobian Ji of bodyName
% ti = Ji*q_dot
%
%   The twist ti is expressed in the body frame

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

    N = sr_info.N;
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage
    
    Rb = R{1};
    Ra = R{2};

    % --- Nkl ---
    Nkl = eye(6*nk, 6*nk);
    for i=2:nk
        A_i = A{2}(:, :, i) ;% A_i_(i-1)
        
        for j=i-1:-1:1       
            if j==i-1
                blkMat = A_i; % Set lower diag to A_i_(i-1)
            else        
                blkMat = Nkl(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl(6*(j+1)-5:6*(j+1), 6*j-5:6*j);
            end
    
            Nkl(6*i-5:6*i, 6*j-5:6*j) = blkMat;
        end
    end
    
    % --- Nd ---
    Nd = zeros(6*nk, nk);
    for i=1:nk
        Nd(6*i-5:6*i, i) = sr_info.P{2}(:, :, i);
    end
    
    
    % --- Nbl ---
    Ab = zeros(6*nk, 6);

    A_0b_b = sr_info.A{1} * [Rb.', zeros(3, 3); zeros(3, 3), eye(3)]; % Base to anchor twist propagation matrix, base frame
    A_0b_k = Ra.' * A_0b_b;  % Base to anchor twist propagation matrix, Appendage frame
    
    A_1b = A{2}(:, :, 1) * A_0b_k; % A_1b = A_10 * A_0b_k
    Ab(1:6, :) = A_1b;
    
    Nbl = Nkl*Ab;
    
    % --- Ndb ---
    Ndb = sr_info.P{1};
    
    % --- Jacobians ---   
    bodyIdx = find(strcmp(sr_info.BodyNames, bodyName));
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

