% Assumiamo che tu abbia già nel workspace A_long e B_ctrl
% Se B_ctrl ha 2 colonne, la prima è l'elevatore (de)

% 1. Definizione delle matrici C per le uscite SISO
% Vogliamo monitorare q (3° stato) e theta (4° stato)
C_q     = [0 0 1 0]; 
C_theta = [0 0 0 1];
D       = 0;

% 2. Creazione dei sistemi nello spazio degli stati (solo per elevatore)
% Selezioniamo solo la prima colonna di B_ctrl
B_de = B_ctrl(:, 2);

sys_q     = ss(A_long, B_de, C_q, D);
sys_theta = ss(A_long, B_de, C_theta, D);

% 3. Calcolo delle Funzioni di Trasferimento (TF)
G_q     = tf(sys_q);     % Pitch Rate / Elevator
G_theta = tf(sys_theta); % Pitch Angle / Elevator

% Visualizzazione dei risultati
disp('Funzione di Trasferimento Pitch Rate q(s)/de(s):');
G_q = minreal(G_q) % Semplifica poli/zeri comuni

disp('Funzione di Trasferimento Pitch Angle theta(s)/de(s):');
G_theta = minreal(G_theta)

% 4. Analisi dei poli per distinguere Fugoide e Corto Periodo
[wn, zeta, poles] = damp(A_long);
disp('Analisi dei modi naturali:');
damp(A_long)