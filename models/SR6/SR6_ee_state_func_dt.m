function xk1 = SR6_ee_state_func_dt(xk, uk) % #codegen
    sr_info = SR6_info();

    Ts = 0.25;

    %% RK4
    M = 5;
    h = Ts/M;
    xk1 = xk;
    for i=1:M
%         h = Ts;
        k1 = sr_ee_state_func(xk1, uk, sr_info);
        k2 = sr_ee_state_func(xk1 + k1 * h / 2, uk, sr_info);
        k3 = sr_ee_state_func(xk1 + k2 * h / 2, uk, sr_info);
        k4 = sr_ee_state_func(xk1 + k3 * h, uk, sr_info);
        xk1 = xk1 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end

    %% ODE Method
    % options = odeset('RelTol', 1e-1, 'AbsTol', 1e-1);
    % [~, x] = ode23(@(t, x) sr_ee_state_func(x, uk, sr_info), [0, Ts], xk', options);
    % xk1 = x(end, :)';

    %% Euler Method (first oder Runge-Kutta method)
    % M = 10;
    % delta = Ts/M;
    % for ct=1:M
    %     xk1 = xk1 + delta*sr_ee_state_func(xk1, uk, sr_info);
    % end
end
