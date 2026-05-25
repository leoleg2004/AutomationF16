function [G_inf, g_inf] = cis(A, B, Fx, fx, Fu, fu, Q, R)
    % Control Invariant Set Iterativo tramite Metodo Matriciale (No Polyhedron)
    [K, ~, ~] = dlqr(A, B, Q, R);
    A_cl = A - B*K;
    Gx = Fx; gx = fx;
    Gu_x = [-K; K]; gu_x = [fu(1:3); fu(4:6)];
    G = [Gx; Gu_x]; g = [gx; gu_x];

    G_inf = G; g_inf = g;
    max_iter = 100; tol = 1e-6;
    for i = 1:max_iter
        G_next = G * (A_cl^i);
        if norm(G_next, inf) < tol
            break;
        end
        G_inf = [G_inf; G_next];
        g_inf = [g_inf; g];
    end
end
