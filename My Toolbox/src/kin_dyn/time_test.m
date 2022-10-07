u = ones(8, 1);
x = zeros(16, 1);

f1 = @()sr_state_func(x, u);
f2 = @()sr_state_func_mex(x, u);

t1 = timeit(f1);
t2 = timeit(f2);

fprintf("Time for matlab func: %f\n", t1)
fprintf("Time for mex func: %f\n", t2)
fprintf("Mex file is %.1f times faster\n", t1/t2)
