function [Nkl, Nd, Nbl, Ndb] = DeNOC(sr_info, A, R)
%DENOC Computes the Decoupled Natural Orthogonal Complement Matrices 
%
%   TODO: Add mat definitions
%
%
%
%   -A          Twist propaagtion matrices {Ab, Am}. A_i_(i-1) in frame i
%                   -Ab: Manipulator anchor to base, anchor frame
%                   -Am: Array w/ manip matrices
%
%   -R         Rot Matrice: {Rb, Ra}
%                   Rb: from base to inertial frame 
%                   Ra: from anchor to base frame 

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

A_ab_b = sr_info.A{1} * [Rb.', zeros(3, 3); zeros(3, 3), eye(3)]; % Base to anchor twist propagation matrix, base frame
A_ab_a = Ra.' * A_ab_b;  % Base to anchor twist propagation matrix, Appendage frame

A_1b = A{2}(:, :, 1) * A_ab_a; % A_1b = A_10 * A_ab_a
Ab(1:6, :) = A_1b;

Nbl = Nkl*Ab;
    

% --- Ndb ---
Ndb = sr_info.P{1};

end

