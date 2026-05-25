function [K, P, Q, R, A_cl] = progetta_LQR_continuo(A, B)
    % Calcolo pesi LQR per tempo continuo
    rho_q = 1;      % Peso sugli stati (modifica questo numero)
    rho_r = 100;    % Peso sugli attuatori (modifica questo numero)
    
    Q = rho_q * eye(size(A,1)); 
    R = rho_r * eye(size(B,2));  
    % Calcolo la K ottimale per il modello continuo
    [K, P, ~] = lqr(A, B, Q, R);
    A_cl = A - B*K;
end
