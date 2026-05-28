% =========================================================================
% Tesi Triennale - LQR Longitudinale F-16
% Ing. Leggeri Leonardo
% =========================================================================
close all
% Inizializzazione Path
startup_project();

%% 1. IMPOSTAZIONE DEL SISTEMA (Solo veri attuatori)
% Estraiamo le dimensioni dalle matrici
nx = size(A_long, 1);
nu_long = size(B_long, 2);

B_ctrl = B_long(:, 1:3); 
D_ctrl = D_long(:, 1:3);
nu = size(B_ctrl, 2); 

%% 2. SINTESI DEL CONTROLLORE LQR 
% =========================================================================
% NOTA SUI PESI Q e R:
% Se vuoi modificare l'aggressività del controllore (pesi Q e R),
% apri il file "funzioni_lqr/progetta_LQR_continuo.m" e modificali 
% =========================================================================
disp('--- Sintesi LQR Continuo ---');
[K, P, Q, R, A_cl] = progetta_LQR_continuo(A_long, B_ctrl);

disp('Guadagno K:');
disp(K);
disp('Autovalori a ciclo chiuso (devono avere parte reale negativa):'); disp(eig(A_cl));

% Creiamo il sistema State-Space a ciclo chiuso (Continuo)
sys_cl = ss(A_cl, zeros(nx, nu), eye(nx), zeros(nx, nu));

% Vettore tempo per la simulazione
t = 0:0.05:10;

% Condizione iniziale estrema (Looping/Candela)
x0 = [0; 0.3; 0; 20]; % theta, q, U, W

% Simulazione della risposta libera (initial) del sistema
[y_sim, t_sim, x_sim] = initial(sys_cl, x0, t);

% Traspongo x_sim per avere gli stati sulle righe (come nel tuo script precedente)
xf = x_sim'; 

% Calcolo dei comandi u generati dall'LQR in ogni istante (u = -K*x)
uf = -K * xf; 

%% 4. GRAFICI (Senza vincoli, con nuovi colori)
rad2deg = 180 / pi; % Fattore di conversione utile per la leggibilità

% Definizione del colore Giallo leggibile per sfondo bianco
colore_giallo = [0.9290, 0.6940, 0.1250]; 

% -------------------------------------------------------------------------
% FIGURA 1: DINAMICA DEGLI STATI (Variabili Separate)
% -------------------------------------------------------------------------
figure('Name', 'Controllo LQR Puro: Stati a Ciclo Chiuso', 'Color', 'w', 'Position', [100, 100, 800, 800])

% 1. Angolo di Beccheggio (theta)
subplot(4, 1, 1)
plot(t, xf(1,:) * rad2deg, 'b', 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1);
ylabel('Ampiezza [deg]', 'Interpreter', 'latex')
title('$\theta$ (Pitch Angle)', 'Interpreter', 'latex')
grid on;

% 2. Rateo di Beccheggio (q)
subplot(4, 1, 2)
plot(t, xf(2,:) * rad2deg, 'r', 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1);
ylabel('Ampiezza [deg/s]', 'Interpreter', 'latex')
title('$q$ (Pitch Rate)', 'Interpreter', 'latex')
grid on;

% 3. Velocità asse X (U)
subplot(4, 1, 3)
plot(t, xf(3,:), 'Color', colore_giallo, 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1);
ylabel('Ampiezza [ft/s]', 'Interpreter', 'latex')
title('$U$ (Velocit\`a Orizzontale)', 'Interpreter', 'latex')
grid on;

% 4. Velocità asse Z (W)
subplot(4, 1, 4)
plot(t, xf(4,:), 'g', 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1);
xlabel('Tempo [s]', 'Interpreter', 'latex') 
ylabel('Ampiezza [ft/s]', 'Interpreter', 'latex')
title('$W$ (Velocit\`a Verticale $\approx \alpha$)', 'Interpreter', 'latex')
grid on;

% -------------------------------------------------------------------------
% FIGURA 2: SFORZO DEGLI ATTUATORI (Senza linee di saturazione)
% -------------------------------------------------------------------------
figure('Name', 'Controllo LQR Puro: Sforzo Attuatori', 'Color', 'w', 'Position', [150, 150, 800, 800])

% 1. Thrust (Spinta del Motore)
subplot(3, 1, 1)
plot(t, uf(1,:), 'b', 'LineWidth', 2); hold on; % Blu
yline(0, 'k--', 'LineWidth', 1);
ylabel('Spinta [lbs]')
title('Azione di Controllo Libera: Manetta (Thrust)')
grid on;

% 2. Elevator (Equilibratore)
subplot(3, 1, 2)
plot(t, uf(2,:) * rad2deg, 'r', 'LineWidth', 2); hold on; % Rosso
yline(0, 'k--', 'LineWidth', 1);
ylabel('Deflessione [deg]')
title('Azione di Controllo Libera: Equilibratore')
grid on;

% 3. Leading Edge Flap (LEF)
subplot(3, 1, 3)
plot(t, uf(3,:) * rad2deg, 'g', 'LineWidth', 2); hold on; % Verde
yline(0, 'k--', 'LineWidth', 1);
xlabel('Tempo [s]') 
ylabel('Deflessione [deg]')
title('Azione di Controllo Libera: Leading Edge Flap')
grid on;

% =========================================================================
% VISUALIZZAZIONE GRAFICA DEI NUOVI POLI (Ciclo Chiuso)
% =========================================================================
sys_cl_map = ss(A_cl, B_ctrl, C_long, D_ctrl); 

figure('Name', 'Mappa Poli-Zeri a Ciclo Chiuso (LQR Continuo)', 'Color', 'w', 'Position', [200, 200, 600, 500]);
pzmap(sys_cl_map);
grid on;

title('Verifica Stabilità LQR: Posizione dei Nuovi Poli');
xlabel('Asse Reale (Velocità di convergenza/divergenza)');
ylabel('Asse Immaginario (Frequenza di oscillazione)');

%% 5. RITRATTO DI FASE E FUNZIONE DI LYAPUNOV (LQR)
% Per visualizzare la stabilità, isoliamo la dinamica a corto periodo 
% variando W (velocità verticale) e q (pitch rate) e fissando U=0, theta=0.

% 5.1 Definizione della griglia spaziale
w_range = linspace(-25, 25, 50);   % ft/s
q_range_rad = linspace(-1.5, 1.5, 50); % rad/s
[W_grid, Q_rad] = meshgrid(w_range, q_range_rad);
Q_deg = Q_rad * rad2deg; % Conversione per asse grafico

% 5.2 Calcolo della funzione di Lyapunov V(x) = x^T * P * x sulla griglia
V_surf = zeros(size(W_grid));
for i = 1:size(W_grid, 1)
    for j = 1:size(W_grid, 2)
        % Fissiamo theta=0 e U=0
        stato_surf = [0; Q_rad(i,j); 0; W_grid(i,j)];
        V_surf(i,j) = stato_surf' * P * stato_surf;
    end
end

% 5.3 Dinamica a ciclo chiuso per ode45
dxdt = @(t, x) A_cl * x;

%% 5. RITRATTO DI FASE E FUNZIONE DI LYAPUNOV 3D (LQR)
% Usa la nuova funzione dedicata per generare il ritratto di fase 2D e
% la funzione di Lyapunov in 3D
plot_lyapunov_lqr(P, A_cl);

