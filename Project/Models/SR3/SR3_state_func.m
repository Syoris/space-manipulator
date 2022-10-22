function dx = SR3_state_func(x, u) % #codegen
    sr_info = SR3_info();

    dx = sr_state_func(x, u, sr_info);
end