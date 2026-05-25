function [H_nsteps, h_nsteps] = controllable_set(Fx, fx, Fu, fu, G_inf, g_inf, A, B, N)
    % Controllable Set N-Step tramite Polyhedron
    nx = size(A, 1);
    nu = size(B, 2);
    H_ii_steps = G_inf;
    h_ii_steps = g_inf;
    Hu = Fu; hu = fu;
    Gx = Fx; gx = fx;

    for ii = 1:N
        A_k_1 = [H_ii_steps * A, H_ii_steps * B;
                 zeros(size(Hu, 1), nx), Hu]; 
        B_k_1 = [h_ii_steps; hu];
        temp = Polyhedron(A_k_1, B_k_1); 
        temp = projection(temp, 1:nx); 
        temp.minHRep(); 
        H_ii_steps = [temp.A; Gx]; 
        h_ii_steps = [temp.b; gx];
    end
    H_nsteps = H_ii_steps;
    h_nsteps = h_ii_steps;
end
