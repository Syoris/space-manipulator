function dx = SR_Val_state_func(x, u) % #codegen
    sr_info = SR_Val_info();

    dx = sr_state_func(x, u, sr_info);
end