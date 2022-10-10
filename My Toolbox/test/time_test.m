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


