function nOk = isNSkewSym(obj)
% checkNMatrix Check if N = H_dot - 2*C is skew-symmetric (N = -N')

    % fprintf("Checking N is skew-symmetric ###\n")

    syms t qm1(t) qm2(t)  
    qm_dot = obj.q_dot(7:end);
    qm = obj.q(7:end);

    q_val = [obj.BaseConfig.Position, obj.BaseConfig.Rot, [obj.JointsConfig.JointPosition]]';
    q_dot_val = [obj.BaseSpeed.TSpeed, obj.BaseSpeed.ASpeed, [obj.JointsSpeed.JointSpeed]]';

    H_t = subs(obj.Hsym, obj.q(1:6), q_val(1:6));
    H_t = subs(H_t, qm, [qm1(t); qm2(t)]);

    H_dt = diff(H_t, t);
    H_dt = subs(H_dt, [diff(qm1(t), t); diff(qm2(t), t)], qm_dot);
    H_dt = subs(H_dt, qm, q_val(7:8));
    
    N = H_dt - 2*subs(obj.Csym, [obj.q; obj.q_dot(1:6)], [q_val; q_dot_val(1:6)]);
    N_val = round(double(subs(N, qm_dot, q_dot_val(7:8))), 3);

    fprintf('N skew-symmetric: ')
    if(N_val == -(N_val'))
        fprintf('OK\n')
        nOk = true;
    else
        fprintf('NOK\n')
        nOk = false;
    end
end