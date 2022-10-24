function cOk = isCOk(obj)
    %isCOk  Check if C Matrix is okay. C matrix choice is not unique. It's valid as long as it
    % respects: sum_j=1:n{ c_ij * q_dot(j)} = sum_j=1:n{ sum_k=1:n{ h_ijk * q_dot(k)*q_dot(j) }}
    % with h_ijk = d/d{q(k)}(h_ij) - 1/2 * d/d{q(i)}(h_jk)

    obj.logger("Checking C validity\n", 'info');
    obj.logger("-- Computing h_ijk--", 'debug');

    cOk = true;

    Hmat = obj.H_symb;
    K = obj.NumActiveJoints + 6;
    h = sym(zeros(K, K, K));

    for i = 1:K
        msg = sprintf('\ti= %i', i);
        obj.logger(msg, 'debug');

        for j = 1:K

            for k = 1:K
                h_ijk = (diff(Hmat(i, j), obj.q_symb(k)) - 0.5 * diff(Hmat(j, k), obj.q_symb(i))) * obj.q_dot_symb(k);
                h(i, j, k) = h_ijk;
            end

        end

    end

    % Check C Matrix
    obj.logger('\n-- C Matrix Check --', 'info');

    Cmat = obj.C_symb;

    for i = 1:K
        t1 = 0;
        t2 = 0;

        for j = 1:K
            t1 = t1 + Cmat(i, j) * obj.q_dot(j);

            for k = 1:K
                t2 = t2 + h(i, j, k) * obj.q_dot(j);
            end

        end

        t1 = round(double(subs(t1, [obj.q_symb; obj.q_dot_symb], [obj.q; obj.q_dot])), 3);
        t2 = round(double(subs(t2, [obj.q_symb; obj.q_dot_symb], [obj.q; obj.q_dot])), 3);

        msg = sprintf(['Row %i:\n ' ...
                        '\t-Computed: %f\n' ...
                    '\t-Check: %f\n'], i, t1, t2)
        obj.logger(msg, 'debug');

        if t1 ~= t2
            cOk = false;
        end

    end

    if cOk
        obj.logger('Matrix C is OK', 'info');
    else
        obj.logger('Matrix C is NOK', 'warning');
    end

end
