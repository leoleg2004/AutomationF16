% =========================================================================
% TUNING MANUALE DEL BECCHEGGIO (Bypassando il pidTuner)
% =========================================================================
disp('--- Analisi del Luogo delle Radici per G_q ---');

% 1. Disegniamo il luogo delle radici per capire l'instabilità
figure;
rlocus(G_q);
title('Luogo delle Radici di G_q');
grid on;
disp('Guarda il grafico: la linea deve spostarsi da destra (instabile) a sinistra (stabile).');

% 2. Progettiamo un controllore PD manuale
% Scegliamo un Kp abbastanza alto per "tirare" il polo instabile a sinistra
% e un Kd per smorzare le oscillazioni. 
% (Nota: i valori esatti dipendono dal tuo modello, prova questi per iniziare)

Kp = -2;  % Prova con -2 o +2 (il segno dipende dal luogo delle radici)
Kd = -0.5; % L'azione derivativa aiuta la stabilità
Ki = 0;   % NIENTE azione integrale!

% Creiamo il controllore
C_q_manuale = pid(Kp, Ki, Kd);

% 3. Calcoliamo il sistema chiuso e verifichiamo i poli VERI
G_q_chiuso = feedback(G_q * C_q_manuale, 1);
disp('Poli del sistema chiuso con tuning manuale:');
disp(pole(G_q_chiuso));

% 4. Simuliamo il gradino in modo sicuro
figure;
step(G_q_chiuso);
title('Risposta al gradino G_q (Tuning Manuale)');

% 1. Pulizia drastica dei poli duplicati (aumentiamo la tolleranza del minreal)
G_q_clean = minreal(G_q, 1e-4);

% Controlliamo che i poli siano tornati ad essere solo 4 (o simili)
disp('Poli puliti di G_q:');
disp(pole(G_q_clean));

% 2. Apriamo il tool definitivo per la progettazione visiva
controlSystemDesigner('rlocus', G_q_clean)