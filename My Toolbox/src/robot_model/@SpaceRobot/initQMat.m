function initQMat(obj)
    Q = sym(zeros(obj.NumActiveJoints + 6, obj.NumActiveJoints + 6));
    Q(7:end, 7:end) = eye(obj.NumActiveJoints);

    S0 = rpy2jac(obj.BaseConfig.Rot);
    [rotM, ~] = tr2rt(obj.Ttree.(obj.BaseName)); % RotM From Base to Inertial frame

    Q(1:3, 1:3) = rotM;
    Q(4:6, 4:6) = transpose(S0);

    obj.Qsym = Q;
end