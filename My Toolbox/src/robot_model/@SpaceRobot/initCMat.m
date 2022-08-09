function C = initCMat(obj)
% nlMatrix  Compute the non-linear velocity term matrix using Christoffel symbols of the first kind.

    K = 6+obj.NumActiveJoints;
    C = sym(zeros(K));
    H = obj.Hsym;
    fprintf('\t Computing C Matrix');
    fprintf('\t Progress: ')
    for i=1:K
        fprintf('.')
        for j=1:K
            c_ij = 0;

            for k=1:K         
                c_ijk = 1/2 * ( diff(H(i, j), obj.q(k)) + diff(H(i, k), obj.q(j)) - diff(H(j, k), obj.q(i)) ) * obj.q_dot(k);
                
                c_ij = c_ij + c_ijk;
            end

            C(i, j) = c_ij;
        end
    end
    fprintf('\n')

    obj.Csym = C;

end