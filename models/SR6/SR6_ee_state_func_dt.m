function xk1 = SR6_ee_state_func_dt(xk, uk) % #codegen
    sr_info = SR6_info();

%     M = 10;
%     delta = Ts/M;
    Ts = 0.5;


    options = odeset('RelTol',1e-1,'AbsTol',1e-1);

    [~, x] = ode23(@(t,x) sr_ee_state_func(x, uk, sr_info), [0, Ts], xk', options);

    xk1 = x(end, :)';

%     for ct=1:M
%         xk1 = xk1 + delta*sr_ee_state_func(xk1, uk, sr_info);
%     end
end