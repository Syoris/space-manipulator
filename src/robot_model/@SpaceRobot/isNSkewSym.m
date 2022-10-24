function nOk = isNSkewSym(obj)
% checkNMatrix Check if N = H_dot - 2*C is skew-symmetric (N = -N')

    % fprintf("Checking N is skew-symmetric ###\n")

    syms t

    qm_t = sym(zeros(obj.NumActiveJoints, 1));
    for i=1:obj.NumActiveJoints
        qm_t(i) = str2sym(sprintf('qm%i(t)', i));
    end

    qm_diff = sym(zeros(obj.NumActiveJoints, 1));
    for i=1:obj.NumActiveJoints
        qm_diff(i) = str2sym(sprintf('diff(qm%i(t), t)', i));
    end

    H_t = subs(obj.H_symb, obj.q_symb(1:6), obj.q0);
    H_t = subs(H_t, obj.q_symb(7:end), qm_t);

    H_dt = diff(H_t, t);
    H_dt = subs(H_dt, qm_diff, obj.q_dot_symb(7:end));
    H_dt = subs(H_dt, qm_t, obj.q(7:end));
    
    N = H_dt - 2*subs(obj.C_symb, [obj.q_symb; obj.q_dot_symb(1:6)], [obj.q; obj.q_dot(1:6)]);
    N_val = round(double(subs(N, obj.q_dot_symb(7:end), obj.q_dot(7:8))), 3);

    fprintf('N skew-symmetric: ')
    if(N_val == -transpose(N_val))
        fprintf('OK\n')
        nOk = true;
    else
        fprintf('NOK\n')
        nOk = false;
    end
end