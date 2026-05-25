% =========================================================================
% Tesi Triennale - MPC Longitudinale F-16
% Simulazione con Conversione Continuo-Discreto (c2d) e Radianti
% Ing. Leggeri Leonardo
% =========================================================================

addpath(fullfile(pwd, 'funzioni_mpc'));

%% 1. Definizione del Sistema (TEMPO CONTINUO)
nx = 4; % Stati: [theta,q,U,W]'
nu = 3; % Ingressi: [T, dele, dlef]'
Ts = 0.05; % Tempo di campionamento in secondi

disp('Conversione del modello da Continuo a Discreto...');
[A_long_ds, B_ctrl_ds] = discretizza_modello(A_long, B_ctrl, nx, nu, Ts);

%% 2. Progetto LQR tramite funzione dedicata
[K, P, Q, R, A_cl] = progetta_LQR(A_long_ds, B_ctrl_ds);

%% 3. Vincoli Fisici (Ampiezza e Rateo) tramite funzione dedicata
[U_min, U_max, dU_min, dU_max, X_min, X_max, Fx, fx, Fu, fu, Gx, gx] = imposta_vincoli(nx, nu, Ts);

%% 4. Calcolo Control Invariant Set e Plot
disp('--- CALCOLO O_INF (Control Invariant Set) ---');
[G_inf, g_inf] = cis(A_long_ds, B_ctrl_ds, Fx, fx, Fu, fu, Q, R);

%% 4b. Plot 3D dei Set Invarianti
plot_cis(G_inf, g_inf);

%% 5. Setup Problema MPC 
N = 30; % Orizzonte predittivo sufficientemente lungo
mpc_prob = setup_mpc(N, nx, nu, A_long_ds, B_ctrl_ds, Q, P, R, U_min, U_max, Gx, gx, G_inf, g_inf);

%% 6. Simulazione MPC Completa 
disp('--- Avvio Ottimizzazione e Simulazione MPC ---');
% Ordine: [theta; q; u; w]
x_iniziale = [deg2rad(25);  % theta: 5 gradi convertiti in rad
              deg2rad(20);  % q: velocità angolare
              20;           % u: +20 ft/s di velocità forward
              20];          % w: velocità verticale
t_sim = 100; 

[storia_x, storia_u] = simula_mpc(mpc_prob, x_iniziale, t_sim, A_long_ds, B_ctrl_ds, dU_max);
disp('Ottimizzazione Riuscita! Il modello è matematicamente solido.');

%% 7. Grafici 
plot_risultati(t_sim, storia_x, storia_u, U_min, U_max);