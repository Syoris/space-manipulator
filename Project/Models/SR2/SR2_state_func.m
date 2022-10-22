function dx = SR2_state_func(x, u) % #codegen
    sr_info = SR2_info();

    dx = sr_state_func(x, u, sr_info);
end