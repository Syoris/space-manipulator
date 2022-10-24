function initCMat(obj, d, simpC)
% nlMatrix  Compute the non-linear velocity term matrix using Christoffel symbols of the first kind.

    K = 6+obj.NumActiveJoints;
    C = sym(zeros(K));
    H = obj.H_symb;

    for i=1:K
        for j=1:K
            d.Message = sprintf('Computing C... (%i, %i)', i, j);
            c_ij = 0;

            for k=1:K         
                c_ijk = 1/2 * ( diff(H(i, j), obj.q_symb(k)) + diff(H(i, k), obj.q_symb(j)) - diff(H(j, k), obj.q_symb(i)) ) * obj.q_dot_symb(k);
                
                c_ij = c_ij + c_ijk;
            end

            C(i, j) = c_ij;
        end
    end

    obj.C_symb = C;

    if simpC
        for i=1:size(C, 1)
            for j=1:size(C, 1)
                d.Message = sprintf('Simplifying C... (%i, %i)', i, j);
                obj.C_symb(i, j) = simplify(obj.C_symb(i, j), 'IgnoreAnalyticConstraints',true,'Seconds',10);
            end
        end
    end

end