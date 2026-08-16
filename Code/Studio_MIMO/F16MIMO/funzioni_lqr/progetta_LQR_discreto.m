% =========================================================================
% Copyright (c) 2026 Ing. Leggeri Leonardo
% Tutti i diritti riservati.
%
% ATTENZIONE: Questo software e il relativo codice sorgente sono di proprietà 
% esclusiva dell'Ing. Leggeri Leonardo. È severamente vietata la copia, 
% la distribuzione, la modifica o la vendita a terzi senza l'esplicito 
% consenso scritto dell'autore. La distribuzione o la vendita non autorizzata 
% costituisce reato ed è perseguibile penalmente secondo le leggi vigenti.
% =========================================================================

function [K, P, Q, R, A_cl] = progetta_LQR_discreto(A, B)
    % =====================================================================
    % Calcolo pesi LQR (Regola di Bryson) coerenti con il modello F-16
    % Ordine stati: [theta (rad); q (rad/s); U (m/s); W (m/s)]
    % Ordine input: [Thrust (N); Elevator (rad); LEF (rad)]
    % =====================================================================
    
    ft2m = 0.3048;
    lbf2N = 4.44822;
    
    % Massimi scostamenti tollerabili per gli stati (Bryson's rule)
    max_theta = 15 * pi/180; % 15 gradi max tollerati per il pitch
    max_q     = 30 * pi/180; % 30 deg/s max tollerati per il pitch rate
    max_U     = 30 * ft2m;   % 30 ft/s convertiti in m/s max per la velocità X
    max_W     = 20 * ft2m;   % 20 ft/s convertiti in m/s max per la velocità Z
    
    Q = diag([1/max_theta^2, 1/max_q^2, 1/max_U^2, 1/max_W^2]);
    
    % Massimi scostamenti tollerabili per gli attuatori
    max_thrust = 5000 * lbf2N;   % 5000 lbs convertite in Newton
    max_elev   = 25 * pi/180;    % 25 gradi max deflessione equilibratore
    max_lef    = 25 * pi/180;    % 25 gradi max deflessione LEF
    
    R = diag([1/max_thrust^2, 1/max_elev^2, 1/max_lef^2]);
    
    % Fattori di tuning per dare più aggressività globale (se necessario)
    rho_q = 20;  % Bilanciamento: molto reattivo ma senza rendere il problema Infeasible
    rho_r = 1;   % Mantiene uno sforzo di controllo ragionevole per non violare i vincoli con x_iniziale elevato 
    
    Q = rho_q * Q;
    R = rho_r * R;
    
    % Sintesi LQR
    fprintf('DEBUG progetta_LQR_discreto:\n');
    fprintf('Size of A: %dx%d\n', size(A,1), size(A,2));
    fprintf('Size of B: %dx%d\n', size(B,1), size(B,2));
    fprintf('Size of Q before dlqr: %dx%d\n', size(Q,1), size(Q,2));
    fprintf('Size of R before dlqr: %dx%d\n', size(R,1), size(R,2));
    
    % Assicuriamoci che Q e R siano delle dimensioni esatte
    Nx = size(A, 1);
    Nu = size(B, 2);
    
    if size(Q, 1) ~= Nx || size(Q, 2) ~= Nx
        disp('ATTENZIONE: Q non è della dimensione corretta! Ricostruzione sicura in corso...');
        Q_new = eye(Nx);
        n_min = min(Nx, size(Q,1));
        Q_new(1:n_min, 1:n_min) = Q(1:n_min, 1:n_min);
        Q = Q_new;
    end
    
    if size(R, 1) ~= Nu || size(R, 2) ~= Nu
        disp('ATTENZIONE: R non è della dimensione corretta! Ricostruzione sicura in corso...');
        R = eye(Nu);
    end
    
    [K, P, ~] = dlqr(A, B, Q, R);
    A_cl = A - B*K;
end
