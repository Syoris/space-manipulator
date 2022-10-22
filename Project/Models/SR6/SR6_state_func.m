function dx = SR6_state_func(x, u) % #codegen
    sr_info = SR6_info();

    dx = sr_state_func(x, u, sr_info);
end