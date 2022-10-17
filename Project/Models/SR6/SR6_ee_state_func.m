function dx = SR6_ee_state_func(x, u) % #codegen
    sr_info = SR6_info();

    dx = sr_ee_state_func(x, u, sr_info);
end