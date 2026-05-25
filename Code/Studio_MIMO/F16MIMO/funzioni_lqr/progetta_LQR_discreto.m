function [K, P, Q, R, A_cl] = progetta_LQR_discreto(A, B)
    % Calcolo pesi LQR per tempo discreto (Regola di Bryson)
    Q = diag([1/20^2, 1/15^2, 1/deg2rad(10)^2, 1/deg2rad(10)^2]); 
    R = diag([1/5000^2, 1/25^2, 1/25^2]);  
    % Calcolo la K ottimale per il modello discreto
    [K, P, ~] = dlqr(A, B, Q, R);
    A_cl = A - B*K;
end
