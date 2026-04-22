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
Q = 1 * eye(nx);
R = 1 * eye(nu);
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
t = 0:Ts:5;

% Condizione iniziale estrema (Looping/Candela)
x0 = [1.5; 1; 10; 50]; % theta, q, U, W

% Simulazione della risposta libera (initial) del sistema
[y_sim, t_sim, x_sim] = initial(sys_cl, x0, t);

% Traspongo x_sim per avere gli stati sulle righe (come nel tuo script precedente)
xf = x_sim'; 

% Calcolo dei comandi u generati dall'LQR in ogni istante (u = -K*x)
uf = -K * xf; 

%% 4. GRAFICI
figure('Name', 'Controllo LQR Puro (Senza Vincoli)', 'Color', 'w')

% Grafico degli Stati
subplot(2, 1, 1)
plot(t, xf(1,:), 'LineWidth', 2); hold on;
plot(t, xf(2,:), 'LineWidth', 2);
plot(t, xf(3,:), 'LineWidth', 2);
plot(t, xf(4,:), 'LineWidth', 2);
legend('\theta (Beccheggio)', 'q (Vel. Beccheggio)', 'U (Vel. X)', 'W (Vel. Z)', 'Location', 'best')
xlabel('Tempo [s]') 
ylabel('Ampiezza Stati')
title('Dinamica degli Stati a Ciclo Chiuso (LQR)')
grid on;

% Grafico dei Comandi (Attuatori)
subplot(2, 1, 2)
plot(t, uf(1,:), 'LineWidth', 2); hold on; % Thrust
plot(t, uf(2,:), 'LineWidth', 2);          % Elevator
plot(t, uf(3,:), 'LineWidth', 2);          % LEF
legend('Thrust', 'Elevator', 'LEF', 'Location', 'best')
xlabel('Tempo [s]') 
ylabel('Comandi (u)')
title('Sforzo di Controllo')
grid on;



% =========================================================================
% VISUALIZZAZIONE GRAFICA DEI NUOVI POLI (Ciclo Chiuso)
% =========================================================================

% 1. Creazione del nuovo sistema "controllato" (ciclo chiuso)
% Sostituiamo la matrice A originale con la nuova dinamica A_cl
sys_cl = ss(A_cl, B_ctrl, C_long, D_ctrl);%sistema in close loop con dinmaica del 
%vento tolta sia dalla matrice B che D

% 2. Generazione della Mappa Poli-Zeri
figure('Name', 'Mappa Poli-Zeri a Ciclo Chiuso (LQR)', 'Position', [200, 200, 600, 500]);
pzmap(sys_cl);
grid on;

% 3. Formattazione grafica per renderlo più leggibile
title('Verifica Stabilità LQR: Posizione dei Nuovi Poli');
xlabel('Asse Reale (Velocità di convergenza/divergenza)');
ylabel('Asse Immaginario (Frequenza di oscillazione)');

% Aggiungiamo una linea marcata sullo zero per evidenziare il limite di stabilità
xline(0, 'k--', 'LineWidth', 1.5);