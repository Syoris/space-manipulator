u = eye(8) * rand(8, 1);
x = eye(16) * rand(16, 1);
xee = eye(28) * rand(28, 1);

f1 = @()sr_state_func(x, u);
f2 = @()sr_state_func_mex(x, u);
f3 = @()sr_ee_state_func(xee, u);
f4 = @()sr_ee_state_func_mex(xee, u);

t1 = timeit(f1);
t2 = timeit(f2);
t3 = timeit(f3);
t4 = timeit(f4);

fprintf("Time for matlab func (joint ode): %f\n", t1)
fprintf("Time for mex func (joint ode): %f\n", t2)
fprintf("Mex file is %.1f times faster\n", t1 / t2)

fprintf("\nTime for matlab func (ee ode): %f\n", t3)
fprintf("Time for mex func (ee ode): %f\n", t4)
fprintf("Mex file is %.1f times faster\n", t3 / t4)

%% h
sr_info = SR2_info();

q = eye(8) * rand(8, 1);
q_dot = eye(8) * rand(8, 1);

% 1 - Kinematics
[Rb, Ra, Rm] = RFunc_SR2(q);
Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays
R = {Rb, Ra, Rm};

% 2 - Kinetics
[t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});
wb = t{1}(4:6);

% 3 - ID
f1 = @()func1(sr_info, t, t_dot, Omega, A, A_dot, Rb);
f2 = @()func2(sr_info, wb, Omega, A, A_dot, R, q_dot);

t1 = timeit(f1);
t2 = timeit(f2);

fprintf("Time for ID: %f\n", t1)
fprintf("Time for C: %f\n", t2)
fprintf("ID is %.1f times faster\n", t2 / t1)
%% Discretization method
clc
Ts = 0.25;
M = 5;

N = 12;
uk = eye(N) * rand(N, 1);
xk = eye(2 * N + 12) * rand(2 * N + 12, 1);
sr_info = SR6_info();

% Check final values
xk1_rk4 = rk4(xk, uk, sr_info, Ts, M);
xk1_ode = odeF(xk, uk, sr_info, Ts);
xk1_rk1 = rk1(xk, uk, sr_info, Ts, M);
% 
% fprintf('RK1\n')
% xk1_rk1(1:6)'
% 
% fprintf('RK4\n')
% xk1_rk4(1:6)'
% 
% fprintf('ODE\n')
% xk1_ode(1:6)'
fprintf("MAX ERROR, RK1: %.5f\n", max(abs(xk1_ode - xk1_rk1)))
fprintf("MAX ERROR, RK4: %.5f\n", max(abs(xk1_ode - xk1_rk4)))


% Time test
f1 = @()rk1(xk, uk, sr_info, Ts, M);
f2 = @()rk4(xk, uk, sr_info, Ts, M);
f3 = @()odeF(xk, uk, sr_info, Ts);

t1 = timeit(f1);
t2 = timeit(f2);
t3 = timeit(f3);

% fprintf("Time for RK4: %f\n", t1)
% fprintf("Time for ODE23: %f\n", t2)

fprintf("\nRK1 is %.1f times faster\n", t3 / t1)
fprintf("RK4 is %.1f times faster\n", t3 / t2)

%% FUNCTIONS
function xk1 = rk4(xk, uk, sr_info, Ts, M)
%     M = 10;
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
end

function xk1 = odeF(xk, uk, sr_info, Ts)
    options = odeset('RelTol', 1e-1, 'AbsTol', 1e-1);
    [~, x] = ode23(@(t, x) sr_ee_state_func(x, uk, sr_info), [0, Ts], xk', options);
    xk1 = x(end, :)';
end

function xk1 = rk1(xk, uk, sr_info, Ts, M)
%     M = 10;
    delta = Ts/M;
    xk1 = xk;
    for ct=1:M
        xk1 = xk1 + delta*sr_ee_state_func(xk1, uk, sr_info);
    end
end

function func1(sr_info, t, t_dot, Omega, A, A_dot, Rb)
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
    h = [tau_b; tau{2}];
end

function func2(sr_info, wb, Omega, A, A_dot, R, q_dot)
    C = CorMat(sr_info, wb, Omega, A, A_dot, R);
    h = C * q_dot;
end
