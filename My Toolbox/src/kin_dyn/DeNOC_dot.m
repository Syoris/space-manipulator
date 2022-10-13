function [Nkl_dot, Nd_dot, Nbl_dot, Ndb_dot] = DeNOC_dot(sr_info, A, A_dot, R, Nkl, wb, Omega)
%DENOC_DOT Computes the derivative of the Decoupled Natural Orthogonal Complement Matrices 
%   TODO: Add mat definitions
%
%
%
%   -A          Twist propaagtion matrices {Ab, Am}. A_i_(i-1) in frame i
%                   -Ab: Manipulator anchor to base, anchor frame
%                   -Am: Array w/ manip matrices
%
%   -A_dot      Derivative of twist propagtion matrices {Ab_dot, Am_dot}. A_i_(i-1) in frame i
%                   -Ab_dot: Manipulator anchor to base, anchor frame
%                   -Am_dot: Array w/ manip matrices
%
%
%   -R         Rot Matrice: {Rb, Ra}
%                   Rb: from base to inertial frame 
%                   Ra: from anchor to base frame 
%
%   -Nkl       Computed in DeNOC func
%
%   -wb         Base angular rate
%
%   -Omega      Omega array {Omega_b, Omega_m}. Omega_m: (6, 6, nk)

nk = sr_info.nk; % Number of bodies in the appendage

Rb = R{1};
Ra = R{2};

% --- Nkl_dot ---
Nkl_dot = zeros(6*nk, 6*nk);
for i=2:nk
    A_dot_i = A_dot{2}(:, :, i) ;% A_dot_i_(i-1)
    
    for j=i-1:-1:1       
        if j==i-1
            blkMat = A_dot_i; % Set lower diag to A_i_(i-1)
        else        
            t1 = Nkl_dot(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl(6*(j+1)-5:6*(j+1), 6*j-5:6*j); % A_dot_i_(j+1) * A_(j+1)_(j)
            t2 = Nkl(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl_dot(6*(j+1)-5:6*(j+1), 6*j-5:6*j); % A_i_(j+1) * A_dot_(j+1)_(j)
            blkMat = t1 + t2;
        end

        Nkl_dot(6*i-5:6*i, 6*j-5:6*j) = blkMat;
    end
end

% --- Nbl_dot ---
Ab = zeros(6*nk, 6);
Ab_dot = zeros(6*nk, 6);

% Base to anchor twist propagation matrix
A_ab_b = sr_info.A{1} * [Rb.', zeros(3, 3); zeros(3, 3), eye(3)]; % Base to anchor twist propagation matrix, base frame
A_ab_a = Ra.' * A_ab_b;  % Base to anchor twist propagation matrix, Appendage frame

A_ab_dot_b = zeros(6, 6);
A_ab_dot_b(1:3, 4:6) = -skew(sr_info.A{1}(1:3, 4:6)*wb);
A_ab_dot_a = Ra.' * A_ab_dot_b;

% Body 1 to anchor
A_1a = A{2}(:, :, 1);
A_1a_dot = A_dot{2}(:, :, 1);

% Body 1 to base
A_1b = A{2}(:, :, 1) * A_ab_a; % A_1b = A_10 * A_ab_a
A_1b_dot = A_1a_dot * A_ab_a + A_1a * A_ab_dot_a; % A_1b_dot = A_1a_dot * A_ab_a + A_1a * A_ab_dot_a

% Base speed matrices
Ab(1:6, :) = A_1b;
Ab_dot(1:6, :) = A_1b_dot;

Nbl_dot = Nkl_dot*Ab + Nkl*Ab_dot;

% --- Nd_dot ---
Nd_dot = zeros(6*nk, nk);
for i=1:3
    Nd_dot(6*i-5:6*i, i) = Omega{2}(:, :, i)*sr_info.P{2}(:, :, i);
end

% Ndb_dot = Omega{1}*sr_info.P{1};
Ndb_dot = zeros(6, 6);
Ndb_dot(4:6, 4:6) = skewSym(wb);
Ndb_dot = Ndb_dot*sr_info.P{1};

end

