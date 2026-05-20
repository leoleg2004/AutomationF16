% =========================================================================
% Tesi Triennale - LQR Longitudinale F-16
% Ing. Leggeri Leonardo
% =========================================================================
close all
clc
warning off all

%% 1. IMPOSTAZIONE DEL SISTEMA (Solo veri attuatori)
% Estraiamo le dimensioni dalle matrici
nx = size(A_long, 1);
nu_long = size(B_long, 2);

%% 2. SINTESI DEL CONTROLLORE LQR
% Prendiamo tutte le righe (:), ma solo le prime 3 colonne (i veri attuatori)
% Lasciamo fuori l'eventuale colonna del vento.
B_ctrl = B_long(:, 1:3); 
%la matrice B totale e l'isnie della Bctrl che prende i ingresso(Thrust,
%Elevator, LEF) 
%e tagliamo fuori la matrice Bwind che ha dentro i contributi del vettore
%vento che però non sono controllabili dal mio lQR.
D_ctrl = D_long(:, 1:3);

% ATTENZIONE: Prendiamo solo i veri attuatori (Thrust, Elevator, LEF)
nu = size(B_ctrl, 2); 
disp('--- Variabili LQR calcolate ---');
disp('Matrice di Riccati P calcolata e pronta per il costo terminale.');

% Pesi logica pura (Aggressivo)
Q = 100* eye(nx);
R = 10* eye(nu);
R_long= 1*eye(nu_long);

% Calcolo del guadagno ottimo K usando SOLO B_ctrl
[K, P, E] = lqr(A_long, B_ctrl, Q, R);
disp('--- Variabili LQR calcolate ---');
disp('Guadagno K:'); disp(K);
disp('Autovalori a ciclo chiuso (devono avere parte reale negativa):'); disp(E);

%% 3. SIMULAZIONE A CICLO CHIUSO
% Creiamo la matrice dinamica a ciclo chiuso A_cl = A - B*K
A_cl = A_long - B_ctrl* K;

% Creiamo il sistema State-Space a ciclo chiuso
% (Non abbiamo ingressi esterni in questa simulazione, solo la condizione iniziale)
sys_cl = ss(A_cl, zeros(nx, nu), eye(nx), zeros(nx, nu));

% Vettore tempo per la simulazione (es. 5 secondi con passo 0.05s, per avere 100 samples)
Ts = 0.05;
t = 0:Ts:30;

% Condizione iniziale estrema (Looping/Candela)
x0 = [0.785; 0.3; 10; 5]; % theta, q, U, W

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
% FIGURA 1: DINAMICA DEGLI STATI (Separati per coerenza di scala)
% -------------------------------------------------------------------------
figure('Name', 'Controllo LQR Puro: Stati a Ciclo Chiuso', 'Color', 'w', 'Position', [100, 100, 800, 600])

% Subplot 1: Angoli e Ratei (Convertiti in Gradi)
subplot(2, 1, 1)
plot(t, xf(1,:) * rad2deg, 'b', 'LineWidth', 2); hold on; % theta (Blu)
plot(t, xf(2,:) * rad2deg, 'r', 'LineWidth', 2);          % q (Rosso)
yline(0, 'k--', 'LineWidth', 1);
legend('\theta (Pitch) [deg]', 'q (Pitch Rate) [deg/s]', 'Location', 'best')
ylabel('Ampiezza [Gradi]')
title('Dinamica Assetto (LQR)')
grid on;

% Subplot 2: Velocità (In ft/s)
subplot(2, 1, 2)
plot(t, xf(3,:), 'Color', colore_giallo, 'LineWidth', 2); hold on; % U (Giallo)
plot(t, xf(4,:), 'g', 'LineWidth', 2);                             % W (Verde)
yline(0, 'k--', 'LineWidth', 1);
legend('U (Vel. X) [ft/s]', 'W (Vel. Z / \approx \alpha) [ft/s]', 'Location', 'best')
xlabel('Tempo [s]') 
ylabel('Ampiezza [ft/s]')
title('Dinamica Velocità (LQR)')
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

figure('Name', 'Mappa Poli-Zeri a Ciclo Chiuso (LQR)', 'Color', 'w', 'Position', [200, 200, 600, 500]);
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

% 5.4 Matrice di condizioni iniziali multiple [theta; q; U; W]
% Vogliamo mostrare la convergenza da diverse "direzioni"
x0_mult = [
    0,   0,    0,   0,    0,   0,    0,   0;      % theta
    1.2, -1.2, 0.5, -0.5, 0.8, -0.8, 0.3, -0.3;   % q (rad/s)
    0,   0,    0,   0,    0,   0,    0,   0;      % U
    20,  -20,  15,  -15, -10,  10,   22,  -22     % W (ft/s)
];

% -------------------------------------------------------------------------
% FIGURA 4: RITRATTO DI FASE 2D CON CURVE DI LIVELLO
% -------------------------------------------------------------------------
figure('Name', 'Ritratto di Fase 2D LQR', 'Color', 'w', 'Position', [250, 250, 700, 500]);
hold on; grid on;

% Disegno le curve di livello di V(x)
contour(W_grid, Q_deg, V_surf, 40, 'LineWidth', 1);
colormap jet;
colorbar;

% Simulo e plotto le traiettorie
for i = 1:size(x0_mult, 2)
    [t_ode, x_ode] = ode45(dxdt, [0 5], x0_mult(:, i));
    
    % Corretto: moltiplico per la variabile rad2deg invece di usare le parentesi
    plot(x_ode(:,4), x_ode(:,2) * rad2deg, 'k', 'LineWidth', 1.5);
    % Segno il punto di partenza
    plot(x_ode(1,4), x_ode(1,2) * rad2deg, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
end

title('Ritratto di Fase e Curve di Livello $V(x)$ (LQR)', 'Interpreter', 'latex');
xlabel('$W$ (Velocità Verticale) [ft/s]', 'Interpreter', 'latex');
ylabel('$q$ (Pitch Rate) [deg/s]', 'Interpreter', 'latex');
axis tight;

% -------------------------------------------------------------------------
% FIGURA 5: FUNZIONE DI LYAPUNOV 3D CON TRAIETTORIE
% -------------------------------------------------------------------------
figure('Name', 'Funzione di Lyapunov 3D - LQR', 'Color', 'w', 'Position', [300, 300, 800, 600]);
hold on; grid on;

% Superficie della funzione costo (bacinella)
surf(W_grid, Q_deg, V_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
colormap jet;

% Proiezione delle traiettorie sulla superficie 3D
for i = 1:size(x0_mult, 2)
    [t_ode, x_ode] = ode45(dxdt, [0 5], x0_mult(:, i));
    
    % V_traiettoria = (x_ode * P) .* x_ode sommato per righe
    V_traiettoria = sum((x_ode * P) .* x_ode, 2); 
    
    % Corretto: moltiplico per la variabile rad2deg invece di usare le parentesi
    plot3(x_ode(:,4), x_ode(:,2) * rad2deg, V_traiettoria, 'k-', 'LineWidth', 2);
    plot3(x_ode(1,4), x_ode(1,2) * rad2deg, V_traiettoria(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
end

title('Funzione di Lyapunov $V(x) = x^T P x$ a Ciclo Chiuso', 'Interpreter', 'latex');
xlabel('$W$ [ft/s]', 'Interpreter', 'latex');
ylabel('$q$ [deg/s]', 'Interpreter', 'latex');
zlabel('$V(x)$', 'Interpreter', 'latex');
view(-35, 35);