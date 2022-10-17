function dx = SR2_ee_state_func(x, u) % #codegen
    sr_info = SR2_info();

    dx = sr_ee_state_func(x, u, sr_info);
end