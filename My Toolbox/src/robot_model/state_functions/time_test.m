u = ones(8, 1);
x = zeros(16, 1);

f1 = @()sr_state_func(x, u);