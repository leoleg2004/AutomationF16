function [U_min, U_max, dU_min, dU_max, X_min, X_max, Fx, fx, Fu, fu, Gx, gx] = imposta_vincoli(nx, nu, Ts)
    % IMPOSTA_VINCOLI Carica i vincoli fisici del sistema
    %   Restituisce i limiti per gli attuatori e per gli stati, oltre
    %   alle matrici dei vincoli di forma poliedrale (Fx*x <= fx).

    % Ingressi: [lb, deg, deg]
    U_min = [-4000; -25; -12];  
    U_max = [10000;  25;  12]; 
    Rate_max = [10000; 60; 25]; 
    dU_max = Rate_max * Ts; 
    dU_min = -dU_max;

    % Stati: theta(rad), q(rad/s), u(ft/s), w(ft/s)
    X_max = [ deg2rad(45);  deg2rad(60);  100;  85]; 
    X_min = [-deg2rad(45); -deg2rad(60); -100; -85];

    Fx = [eye(nx); -eye(nx)]; 
    fx = [X_max; -X_min];
    Fu = [eye(nu); -eye(nu)]; 
    fu = [U_max; -U_min];
    
    Gx = Fx; 
    gx = fx; % Per retrocompatibilità del codice MPC
end
