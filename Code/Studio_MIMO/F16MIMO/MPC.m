% =========================================================================
% Tesi Triennale - MPC Longitudinale F-16
% Simulazione con Conversione Continuo-Discreto (c2d) e Radianti
% Ing. Leggeri Leonardo
% =========================================================================

% Inizializzazione Path
% Aggiungiamo prima la cartella locale Startup in cima al path per evitare
% conflitti con startup_project di altri progetti (es. Tre Camere)
addpath(fullfile(pwd, 'Startup'), '-begin');
startup_project();

% =========================================================================
% TABS AUTOMATICI: Inserisce tutti i grafici in un unico pannello a schede
% =========================================================================
set(0, 'DefaultFigureWindowStyle', 'docked');

%% 1. Definizione del Sistema (TEMPO CONTINUO)
nx = 4; % Stati: [theta,q,U,W]'
nu = 3; % Ingressi: [T, dele, dlef]'
Ts = 0.1; % Tempo di campionamento in secondi

disp('Conversione del modello da Continuo a Discreto...');
[A_long_ds, B_ctrl_ds] = discretizza_modello(A_long, B_ctrl, nx, nu, Ts);

%% 2. Progetto LQR tramite funzione dedicata
% =========================================================================
% NOTA SUI PESI Q e R:
% Se vuoi modificare l'aggressività del controllore (pesi Q e R),
% apri il file "funzioni_lqr/progetta_LQR_discreto.m" e modificali lì dentro
% =========================================================================
disp('Progetto LQR discreto tramite funzione dedicata...');
[K, P, Q, R, A_cl] = progetta_LQR_discreto(A_long_ds, B_ctrl_ds);

    % I vincoli FISICI assoluti dell'F-16:
    % Thrust: [1000, 19000] lbf -> Newton
    % Elevator: [-25, 25] gradi
    % Flaps (LEF): [0, 25] gradi
    lbf2N = 4.44822;
    ft2m = 0.3048;
    
    U_min_abs = [1000 * lbf2N; -25 * pi/180; 0];  
    U_max_abs = [19000 * lbf2N; 25 * pi/180; 25 * pi/180]; 

    % Poiché l'MPC ragiona in "perturbazioni" (u_err), i vincoli su u_err 
    % devono essere sottratti del punto di trim (best_u)
    U_min = U_min_abs - [best_u(1); best_u(2); best_u(3)];
    U_max = U_max_abs - [best_u(1); best_u(2); best_u(3)];

    Rate_max = [10000 * lbf2N; 60 * pi/180; 25 * pi/180];

    % Stati: theta(rad), q(rad/s), u(m/s), w(m/s)
    X_max = [ 45 * pi/180;  60 * pi/180;  100 * ft2m;  85 * ft2m]; 
    X_min = [-45 * pi/180; -60 * pi/180; -100 * ft2m; -85 * ft2m];

[Fx, fx, Fu, fu, dU_min, dU_max, Gx, gx] = imposta_vincoli(X_min, X_max, U_min, U_max, Rate_max, Ts);

% Riferimento di Default (Origine del sistema linearizzato)
x_ref = zeros(nx, 1);
u_ref = zeros(nu, 1);

% =========================================================================
% ESEMPIO con target non nullo
% Affinché l'MPC funzioni correttamente sul modello linearizzato, il nuovo 
% target deve essere un punto di equilibrio valido: x_ref = A_ds*x_ref + B_ds*u_ref
%
%u_ref = [100; 2 * pi/180; 0]; % Es: +1000 lbs di spinta e 2 gradi di equilibratore
 %x_ref = (eye(nx) - A_long_ds) \ (B_ctrl_ds * u_ref); % Stato di equilibrio esatto associato
% =========================================================================

%% 4. Calcolo Control Invariant Set e Plot
disp('--- CALCOLO del Control Invariant Set ---');
[G_inf, g_inf] = cis(A_long_ds, B_ctrl_ds, x_ref, u_ref, Fx, fx, Fu, fu, Q, R);

%% 5. Setup Problema MPC 
N = 40; % Orizzonte predittivo 
mpc_prob = setup_mpc(N, nx, nu, A_long_ds, B_ctrl_ds, Q, P, R, U_min, U_max, Gx, gx, G_inf, g_inf, x_ref, u_ref);

%% 4b. Plot 3D dei Set Invarianti e Feasible Set N-Step
% Inizializza Dashboard
tabs = crea_dashboard_mpc();
plot_cis(G_inf, g_inf, x_ref, mpc_prob, A_long_ds, dU_max, tabs.cis_3d);

%% 6. Simulazione MPC Completa 
disp('--- Avvio Ottimizzazione e Simulazione MPC ---');
% Ordine: [theta; q; u; w]
x_iniziale = [25 * (pi/180);  % theta: +25 gradi (cabrata)
              3 * (pi/180);  % q: +20 gradi/s (velocità di beccheggio)
              2;             % u: +20 ft/s (deviazione velocità forward)
              5];            % w: +20 ft/s (deviazione velocità verticale)
t_sim = 100; % tempo di simulazione

[storia_x, storia_u, storia_costo] = simula_mpc(mpc_prob, x_iniziale, t_sim, A_long_ds, B_ctrl_ds, dU_max);
disp('Ottimizzazione Riuscita. Il modello è matematicamente solido.');

disp('---------------------------------------------------');
disp('VERIFICA RAGGIUNGIMENTO TARGET (Ultimo Step)');
disp('Stato Finale Raggiunto (storia_x(:,end)):');
disp(storia_x(:,end));
disp('Target Desiderato (x_ref):');
disp(x_ref);
disp('Errore Assoluto [theta; q; U; W]:');
disp(abs(storia_x(:,end) - x_ref));
disp('---------------------------------------------------');

%% 7. Grafici 
plot_risultati(t_sim, storia_x, storia_u, U_min, U_max, x_ref, u_ref, tabs.stati, tabs.attuatori);

%% 8. Plot Funzionale di Costo MPC 3D
plot_mpc_cost_3d(mpc_prob, A_long_ds, dU_max, storia_x, storia_costo, Ts, tabs.cost_3d);