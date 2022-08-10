%% Initialization of H and C
% clear all
clc
% load 'SC_2DoF_old.mat'

% Spacecraft State
qm_val=[pi/6; -pi/4];
r0_val = [0.5; 0.2; 1];
delta0_val = [0; 0; 0];

r0_dot_val = [0; 0; 0];
w0_val = [0; 0; 0];
qm_dot_val = [4;-1]*pi/180; %Joint velocities

runH = false;
runC = false;
runCCheck = true;

% Initial condition
syms 't' 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1' 'qm2'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm_dot1' 'qm_dot2'

R0_val = rpy2r(delta0_val');
r0= [Rx;Ry;Rz];
delta0 = [r;p;y];
R0 = rpy2r(delta0');
qm= [qm1; qm2];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm_dot1; qm_dot2];
qm_dot = [qm_dot1; qm_dot2];

q_val = [r0_val; delta0_val; qm_val];
q_dot_val = [r0_dot_val; w0_val; qm_dot_val];

% SPART
filename='SC_2DoF.urdf';
[robotSpart,robot_keys] = urdf2robot(filename);

% % Spacecraft state
% sc.JointsConfig = qm_val';
% sc.JointsSpeed = qm_dot_val';
% sc.BaseConfig = [r0_val'; delta0_val'];
% sc.BaseSpeed = [r0_dot_val'; w0_val'];
%% H
if runH
    sc.initMats();
    
    tic
    sc.initMassMat();
    % H1 = sc.Hsym;
    fprintf('H1 computed in: ')
    toc
    
    H2 = zeros(6+sc.NumActiveJoints);
    
    % Base
    tic
    Jacobians = sc.comJacobiansBase();
    Jv_B = Jacobians.(sc.BaseName)(1:3, :); % Base speed jacobian
    Jw_B = Jacobians.(sc.BaseName)(4:6, :); % Base rotation jacobian
    inertias = sc.getInertiaM();
    
    bV = sc.Base.Mass*(Jv_B'*Jv_B);
    bW = Jw_B'*inertias.(sc.BaseName)*Jw_B;
    
    H2 = H2 + bV + bW;
    
    for i=1:sc.NumLinks
        link = sc.Links{i};
        joint = link.Joint;
        
        % Compute for active joints
        if joint.Q_id ~= -1   
    %         disp(link.Name)
            Jv_i = Jacobians.(link.Name)(1:3, :);
            Jw_i = Jacobians.(link.Name)(4:6, :);
            I_i = inertias.(link.Name);
            
            iv_1 = link.Mass*(Jv_B'*Jv_B);
            iv_2 = 2*link.Mass*(Jv_B'*Jv_i);
            iv_2_2 = link.Mass*( Jv_B'*Jv_i + Jv_i'*Jv_B);
            iv_3 = link.Mass*(Jv_i'*Jv_i);
            iw = Jw_i' * I_i * Jw_i;
            
            H2 = H2 + iv_1 + iv_2_2 + iv_3 + iw;
        end
    end
    fprintf('H2 computed in: ')
    toc
    
    % Set Robot Config
    sc.JointsConfig = qm_val';
    sc.JointsSpeed = qm_dot_val';
    sc.BaseConfig = [r0_val'; delta0_val'];
    sc.BaseSpeed = [r0_dot_val'; w0_val'];
    
    % Print Results
    fprintf('\n##### Mass Matrix #####\n')
    fprintf('--- Class ---\n');
%     fprintf('\tSymbolic\n');
%     tic
%     disp(sc.Hsym)
%     toc
    fprintf('\n\nVals\n')
    tic
    disp(sc.getH())
    toc
    
    fprintf('\n--- Function ---\n');
%     fprintf('\tSymbolic\n');
%     tic
%     disp(H2)
%     toc
    fprintf('\n\nVals\n')
    tic
    H2_val = double(subs(H2, sc.q, q_val));
    disp(H2_val)
    toc
end
%% C - Non-Linear Effect 
if runC
    sc.initMats();
    
    % Class
    tic
    sc.initCMat();
    fprintf('C1 computed in: ')
    toc
    
    
    % Function
    H = sc.Hsym;
    tic
    K = 6+sc.NumActiveJoints;
    C2 = sym(zeros(K));
    
    fprintf('\n\tComputing C...\n');
    for i=1:K
        fprintf('\t i=%i\n', i);
        for j=1:K
            c_ij = 0;
    
            for k=1:K         
                c_ijk = 1/2 * ( diff(H(i, j), q(k)) + diff(H(i, k), q(j)) - diff(H(j, k), q(i)) ) * q_dot(k);
                
                c_ij = c_ij + c_ijk;
            end
    
            C2(i, j) = c_ij;
        end
    end
    fprintf('C2 computed in: ')
    toc
    
    % Set Robot Config
    sc.JointsConfig = qm_val';
    sc.JointsSpeed = qm_dot_val';
    sc.BaseConfig = [r0_val'; delta0_val'];
    sc.BaseSpeed = [r0_dot_val'; w0_val'];
    
    % Print Results
    fprintf('\n##### C Matrix #####\n')
    fprintf('--- Class ---\n');
%     fprintf('\tSymbolic\n');
%     tic
%     disp(sc.Csym)
%     toc
    
    fprintf('\n\nVals\n')
    tic
    Ctest = sc.getC();   
    disp(sc.getC())
    toc
    
    fprintf('\n--- Function ---\n');
%     fprintf('\tSymbolic\n');
%     tic
%     disp(C2)
%     toc
    
    fprintf('\n\nVals\n')
    tic
    C2_val = double(subs(C2, [q; q_dot], [q_val; q_dot_val]));  
    disp(C2_val)
%     toc
end

%% C Matrix Check
if runCCheck
    % Skew Sym
    fprintf("### Checking N is skew-symmetric ###\n")
    syms t qm1(t) qm2(t)  
    H_t = subs(sc.Hsym, q(1:6), q_val(1:6));
    H_t = subs(H_t, q(7:8), [qm1(t); qm2(t)]);

    H_dt = diff(H_t, t);
    H_dt = subs(H_dt, [diff(qm1(t), t); diff(qm2(t), t)], qm_dot);
    H_dt = subs(H_dt, q(7:8), q_val(7:8));
    
    N = H_dt - 2*subs(sc.Csym, [q; q_dot(1:6)], [q_val; q_dot_val(1:6)]);
    N_val = round(double(subs(N, q_dot(7:8), q_dot_val(7:8))), 3);

    fprintf('N skew-symmetric: ')
    if(N_val == -(N_val'))
        fprintf('OK\n')
    else
        fprintf('NOK\n')
    end

    % Validity
    % h_ijk computing
    fprintf("\n### Checking C validity ###\n")
    fprintf("-- Computing h_ijk--\n")
    H = sc.Hsym;
    h = sym(zeros(K, K, K));
    for i=1:K
        fprintf('\t i=%i\n', i);
        for j=1:K
    
            for k=1:K         
                h_ijk = ( diff(H(i, j), q(k)) - 0.5*diff(H(j, k), q(i)) ) * q_dot(k);
                h(i, j, k) = h_ijk;
            end
        end
    end
    
    % Check C Matrix
    fprintf('\n-- C Matrix Check --')
    C = sc.Csym;
    for i =1:K
        t1 = 0;
        t2 = 0;
    
        for j=1:K   
            t1 = t1 + C(i, j)*q_dot(j);
            for k=1:K
                t2 = t2 + h(i, j, k)*q_dot(j);
            end
        end
    
        t1 = round(double(subs(t1, [q; q_dot], [q_val; q_dot_val])), 3);
        t2 = round(double(subs(t2, [q; q_dot], [q_val; q_dot_val])), 3);
    
        fprintf(['\nRow %i:\n ' ...
                 '\t-Computed: %f\n' ...
                 '\t-Check: %f\n'], i, t1, t2);
        assert(t1==t2, 'Invalid term for row %i', i);
    end

    fprintf("\nMatrix C is OK\n")
end