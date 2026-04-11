% =========================================================================
% MINIMIZZAZIONE DEL SISTEMA (Raggiungibilità e Osservabilità)
% =========================================================================
disp('--- Generazione Modello State-Space (LTI) Originale ---');
sys_long = ss(A_long, B_long, C_long, D_long);
sys_long.StateName = {'theta', 'q', 'U', 'W'};
sys_long.InputName = {'Thrust', 'Elevator', 'LE_flap', 'Dist_x', 'Dist_z'};
sys_long.OutputName = {'Vt_out', 'alpha_out', 'q_out', 'xbdd', 'zbdd', 'nz'};
disp('--- Estrazione della Realizzazione Minima (minreal) ---');
% Il comando minreal cancella le dinamiche non osservabili/raggiungibili
% ed elimina le coppie polo-zero coincidenti.
sys_long_min = minreal(sys_long);
disp('--- Calcolo Matrice Funzioni di Trasferimento MINIMA G(s) ---');
G_long_min = tf(sys_long_min);

% =========================================================================
% CALCOLO DI POLI E ZERI
% =========================================================================
disp('--- Poli del Sistema Minimo (Autovalori effettivi) ---');
% I poli del sistema MIMO (coincidono con il denominatore comune)
poli_minimi = pole(sys_long_min);
disp(poli_minimi);
disp('--- Zeri di Trasmissione del Sistema MIMO ---');
% Gli zeri di un sistema MIMO non sono banali da calcolare come nei SISO.
% Il comando tzero calcola gli zeri di trasmissione dell'intera matrice.
zeri_mimo = tzero(sys_long_min);
disp(zeri_mimo);

%%
% =========================================================================
% 1. PREPARAZIONE DELL'IMPIANTO (Il vero sistema controllabile 3x3)
% =========================================================================
disp('--- Preparazione dell''Impianto MIMO 3x3 (Solo Attuatori Reali) ---');
% Seleziono  le 3 uscite prioritarie (Vt, alpha, q)
uscite_ctrl = [1, 2, 3]; 
% Seleziono  i 3 attuatori (Thrust, Elevator, LE_flap)
ingressi_ctrl = [1, 2, 3]; 

% Estraiamo la sottomatrice 3x3 in forma di Funzione di Trasferimento
G_3x3 = G_long_min(uscite_ctrl, ingressi_ctrl);

% Estraiamo la sottomatrice 3x3 in forma State-Space
sys_quadrato_3x3 = sys_long_min(uscite_ctrl, ingressi_ctrl);

% =========================================================================
% 2. SINTESI DEL DISACCOPPIATORE STATICO 3x3 (W_static)
% =========================================================================
disp('--- Calcolo del Pre-Compensatore Statico ---');
% Estraiamo la matrice dei guadagni a regime stazionario
K_impianto = dcgain(G_3x3);

% Inversione 
W_static = inv(K_impianto);
disp('Matrice W_static 3x3 calcolata con successo.');

% =========================================================================
% 3. APPLICAZIONE IN "STATE-SPACE" E RICONVERSIONE
% =========================================================================
disp('--- Applicazione del Compensatore in Spazio di Stato ---');
sys_dec = sys_quadrato_3x3 * W_static;
sys_dec = minreal(sys_dec);

% Passiamo alle Funzioni di Trasferimento pulite
G_dec_pulita = tf(sys_dec);

% VERIFICA DI SICUREZZA: 
K_nuovo = dcgain(G_dec_pulita);
disp('Matrice dei Guadagni Statici del nuovo sistema (Deve essere un''Identità 3x3):');
disp(round(K_nuovo, 4));

% =========================================================================
% 4. ESTRAZIONE DEI 3 CANALI INDIPENDENTI PER IL PID TUNER
% =========================================================================
disp('--- Estrazione dei canali puliti per il progetto dei PID ---');
G_Vt    = G_dec_pulita(1,1); %  PID della Velocità
G_alpha = G_dec_pulita(2,2); %  PID dell'Angolo di attacco
G_q     = G_dec_pulita(3,3); %  PID del Pitch rate

disp('Operazione completata con successo! Matrice 3x3 disaccoppiata.');
%% Calcolo della Raggiungibilità / Controllabilità
% Otteniamo il numero di variabili di stato
n = size(A_long, 1);       

% 1. Calcolo della matrice di Raggiungibilità (Controllabilità)
R = ctrb(A_long, B_long);

% 2. Calcolo del rango della matrice
rango_R = rank(R);    

disp('--- Analisi di Raggiungibilità/Controllabilità ---');
% disp('Matrice di Raggiungibilità R:'); 
% disp(R); % Decommenta se vuoi stampare a schermo l'intera matrice

% 3. Verifica della completa raggiungibilità
% Il sistema è completamente raggiungibile se il rango della matrice R 
% è uguale al numero di variabili di stato n.
if rango_R == n
    disp(['✅ Il sistema è COMPLETAMENTE raggiungibile (Rango R = ', num2str(rango_R), ').']);
else
    disp('❌ Il sistema NON è completamente raggiungibile (rango incompleto).');
    disp(['Il rango della matrice R è ', num2str(rango_R), ' invece di ', num2str(n), '.']);
end

%% Calcolo della Raggiungibilità / Controllabilità
% Otteniamo il numero di variabili di stato
n = size(A_long, 1);       

% 1. Calcolo della matrice di Raggiungibilità (Controllabilità)
R = ctrb(A_long, B_long);

% 2. Calcolo del rango della matrice
rango_R = rank(R);    

disp('--- Analisi di Raggiungibilità/Controllabilità ---');
disp('Matrice di Raggiungibilità R:'); 
disp(R); % Ora la matrice verrà stampata a schermo

% 3. Verifica della completa raggiungibilità
if rango_R == n
    disp(['✅ Il sistema è COMPLETAMENTE raggiungibile (Rango R = ', num2str(rango_R), ').']);
else
    disp('❌ Il sistema NON è completamente raggiungibile (rango incompleto).');
    disp(['Il rango della matrice R è ', num2str(rango_R), ' invece di ', num2str(n), '.']);
end

%% Calcolo dell'Osservabilità
% 1. Calcolo della matrice di Osservabilità
O = obsv(A_long, C_long);

% 2. Calcolo del rango della matrice
rango_O = rank(O);    

disp(' '); % Riga vuota per separare l'output
disp('--- Analisi di Osservabilità ---');
disp('Matrice di Osservabilità O:'); 
disp(O); % Ora la matrice verrà stampata a schermo

% 3. Verifica della completa osservabilità
if rango_O == n
    disp(['✅ Il sistema è COMPLETAMENTE osservabile (Rango O = ', num2str(rango_O), ').']);
else
    disp('❌ Il sistema NON è completamente osservabile (rango incompleto).');
    disp(['Il rango della matrice O è ', num2str(rango_O), ' invece di ', num2str(n), '.']);
end