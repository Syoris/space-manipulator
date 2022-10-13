u = eye(8)*rand(8, 1);
x = eye(16)*rand(16, 1);
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
fprintf("Mex file is %.1f times faster\n", t1/t2)

fprintf("\nTime for matlab func (ee ode): %f\n", t3)
fprintf("Time for mex func (ee ode): %f\n", t4)
fprintf("Mex file is %.1f times faster\n", t3/t4)

%% h
sr_info = SR2_info();

q = eye(8)*rand(8, 1);
q_dot = eye(8)*rand(8, 1);

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
fprintf("ID is %.1f times faster\n", t2/t1)

function func1(sr_info, t, t_dot, Omega, A, A_dot, Rb)
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
    h = [tau_b; tau{2}];
end

function func2(sr_info, wb, Omega, A, A_dot, R, q_dot)
    C = CorMat(sr_info, wb, Omega, A, A_dot, R);
    h = C*q_dot;
end
