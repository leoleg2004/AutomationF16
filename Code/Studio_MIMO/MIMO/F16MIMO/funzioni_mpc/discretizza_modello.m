function [A_long_ds, B_ctrl_ds] = discretizza_modello(A_long, B_ctrl, nx, nu, Ts)
    % DISCRETIZZA_MODELLO Converte il sistema da tempo continuo a discreto
    sys_c = ss(A_long, B_ctrl, eye(nx), zeros(nx, nu));
    sys_d = c2d(sys_c, Ts, 'zoh'); % Zero-Order Hold
    A_long_ds = sys_d.A;
    B_ctrl_ds = sys_d.B;
end
