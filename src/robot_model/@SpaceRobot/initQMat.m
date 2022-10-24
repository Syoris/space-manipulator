function initQMat(obj, d)
    d.Message = sprintf('Computing Q...');

    Q = sym(zeros(obj.NumActiveJoints + 6, obj.NumActiveJoints + 6));
    Q(7:end, 7:end) = eye(obj.NumActiveJoints);

    S0 = rpy2jac((obj.q_symb(4:6)).');
    [rotM, ~] = tr2rt(obj.Ttree_symb.(obj.BaseName)); % RotM From Base to Inertial frame

    Q(1:3, 1:3) = rotM;
    Q(4:6, 4:6) = transpose(S0);

    obj.Q_symb = Q;

    for i=1:size(Q, 1)
        for j=1:size(Q, 1)
            d.Message = sprintf('Simplifying Q... (%i, %i)', i, j);
            obj.Q_symb(i, j) = simplify(obj.Q_symb(i, j), 'IgnoreAnalyticConstraints',true,'Seconds',10);
        end
    end
end