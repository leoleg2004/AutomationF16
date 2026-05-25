function [K, P, Q, R, A_cl] = progetta_LQR(A, B)
    % Calcolo pesi LQR
    Q = diag([1/20^2, 1/15^2, 1/deg2rad(10)^2, 1/deg2rad(10)^2]); 
    R = diag([1/5000^2, 1/25^2, 1/25^2]);  
    %calcolo la K per algortimo di convergenza del set e la P come
    %osluzione per la funzione di ljapunov dell'LQR
    [K, P, ~] = dlqr(A, B, Q, R);
    A_cl = A - B*K;
end
