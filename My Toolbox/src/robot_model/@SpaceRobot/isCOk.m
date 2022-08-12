function cOk = isCOk(obj, verbose)
%isCOk  Check if C Matrix is okay. C matrix choice is not unique. It's valid as long as it 
% respects: sum_j=1:n{ c_ij * q_dot(j)} = sum_j=1:n{ sum_k=1:n{ h_ijk * q_dot(k)*q_dot(j) }}
% with h_ijk = d/d{q(k)}(h_ij) - 1/2 * d/d{q(i)}(h_jk)
    if nargin==1
        verbose = false;
    end
    if verbose
        fprintf("Checking C validity\n")
        fprintf("-- Computing h_ijk--\n")
    end

    cOk = true;

    Hmat = obj.H_symb;
    K = obj.NumActiveJoints + 6;
    h = sym(zeros(K, K, K));
    if verbose
        fprintf('\t i=');
    end
    for i=1:K
        if verbose 
            fprintf('%i, ', i);
        end
        for j=1:K
    
            for k=1:K         
                h_ijk = ( diff(Hmat(i, j), obj.q_symb(k)) - 0.5*diff(Hmat(j, k), obj.q_symb(i)) ) * obj.q_dot_symb(k);
                h(i, j, k) = h_ijk;
            end
        end
    end
    if verbose 
        fprintf('\n');
    end
    
    % Check C Matrix
    if verbose 
        fprintf('\n-- C Matrix Check --') 
    end
    
    Cmat = obj.C_symb;

    for i =1:K
        t1 = 0;
        t2 = 0;
    
        for j=1:K   
            t1 = t1 + Cmat(i, j)*obj.q_dot(j);
            for k=1:K
                t2 = t2 + h(i, j, k)*obj.q_dot(j);
            end
        end
    
        t1 = round(double(subs(t1, [obj.q_symb; obj.q_dot_symb], [obj.q; obj.q_dot])), 3);
        t2 = round(double(subs(t2, [obj.q_symb; obj.q_dot_symb], [obj.q; obj.q_dot])), 3);
        
        if verbose
            fprintf(['\nRow %i:\n ' ...
                    '\t-Computed: %f\n' ...
                    '\t-Check: %f\n'], i, t1, t2);
        end
        if t1~=t2
            isOk = false;
        end
    end

    if verbose 
        fprintf("\nMatrix C is OK\n")
    end

end