function J_dot = Jacobian_dot(bodyName, sr_info, A, A_dot, R, wb, Omega)
%JACOBIAN_DOT Computes jacobian derivative, Ji_dot,  of bodyName
% ti_dot = Ji*q_ddot + Ji_dot * q_dot
%
%   Accel expressed in the body frame 
n = sr_info.n;
nk = sr_info.nk; % Number of bodies in the appendage
    

[Nkl, Nd, Nbl, Ndb] = DeNOC(sr_info, A, R);  

[Nkl_dot, Nd_dot, Nbl_dot, Ndb_dot] = DeNOC_dot(sr_info, A, A_dot, R, Nkl, wb, Omega);  


% --- Jacobians derivative ---
bodyIdx = find(strcmp(sr_info.BodyNames, bodyName));
assert(bodyIdx>0, 'ERROR: Invalid body name')

i = bodyIdx;

Jb_dot = Nbl_dot(6*i-5:6*i, :) * Ndb + Nbl(6*i-5:6*i, :) * Ndb_dot;
Jk_dot_full = Nkl_dot(6*i-5:6*i, :) * Nd + Nkl(6*i-5:6*i, :)*Nd_dot;
    
% Remove fix joints
Jk_dot = zeros(6, n);
for i=1:nk
    jnt_idx = sr_info.jnt_idx(i);
    if jnt_idx > 0
        Jk_dot(:, jnt_idx) = Jk_dot_full(:, i);
    end
end

J_dot = [Jb_dot, Jk_dot]; 

end

