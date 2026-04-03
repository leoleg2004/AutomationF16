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
% disaccoppiamento in avanti 
% Scegliamo tutte e 5 le leve, ma scartiamo un'uscita (es. la sesta, 'nz')
G_quadrata = G_long_min(1:5, :); % Prende le prime 5 uscite e tutti e 5 gli ingressi

% Ora è una 5x5! Possiamo fare l'inversa classica
K_Static = dcgain(G_quadrata);
W = inv(K_Static); 

% Applichiamo il disaccoppiatore
G_disaccoppiataInAvanti = G_quadrata * W;



%%
% disaccopiamento con pseudoinversa
% 1. Calcola la matrice dei guadagni statici di tutto il sistema 6x5
K_totale = dcgain(G_long_min);

% 2. Crea il disaccoppiatore in avanti usando la pseudoinversa (pinv)
W = pinv(K_totale);

% 3. Applica il disaccoppiatore all'impianto originale
G_disaccoppiataPseudoInversa = G_long_min * W;

% Mostriamo il risultato per verifica
disp('--- Matrice di Disaccoppiamento W (Pseudoinversa) ---');
disp(W);
% =========================================================================
% VISUALIZZAZIONE GRAFICA
% =========================================================================
% Mappa Poli-Zeri per verificare la stabilità (tutte le X a sinistra dell'asse y)
figure('Name', 'Mappa Poli-Zeri (Minimal Realization)');
pzmap(sys_long_min);
grid on;
title('Mappa Poli-Zeri del Sistema Longitudinale Minimo');

% =========================================================================
% VERIFICA FINALE: RGA SUL SISTEMA DISACCOPPIATO
% =========================================================================
disp('--- Verifica RGA su G_disaccoppiataPseudoInversa ---');

% 1. Calcoliamo il guadagno statico e l'RGA
K_dec = dcgain(G_disaccoppiataPseudoInversa);
RGA_dec = K_dec .* (pinv(K_dec).');

% Nomi FISSI per le uscite (Asse Y) - Esattamente uguali a prima!
nomi_uscite = {'Vt_out', 'alpha_out', 'q_out', 'xbdd', 'zbdd', 'nz'};
% Nomi dei comandi VIRTUALI (Asse X) - Sono 6 per la matrice quadrata
nomi_comandi_virtuali = {'Cmd_Vt', 'Cmd_alpha', 'Cmd_q', 'Cmd_xbdd', 'Cmd_zbdd', 'Cmd_nz'};

% Stampiamo la Tabella a schermo
RGA_table = array2table(round(RGA_dec, 4), 'RowNames', nomi_uscite, 'VariableNames', nomi_comandi_virtuali);
disp('Matrice RGA Disaccoppiata:');
disp(RGA_table);

% Creiamo la figura a destra per il confronto
figure('Name', 'Analisi RGA Post-Disaccoppiamento', 'Position', [820, 200, 700, 500]);
h2 = heatmap(nomi_comandi_virtuali, nomi_uscite, RGA_dec);

% Formattazione Heatmap
h2.Title = 'RGA Sistema Disaccoppiato (Virtuale)';
h2.XLabel = 'Comandi Virtuali (Reference)';
h2.YLabel = 'Uscite Fisiche (Sensori)';
colormap(h2, parula); 
h2.CellLabelFormat = '%.2f';

% STESSA IDENTICA SCALA DEL PRIMO GRAFICO
h2.ColorLimits = [-1, 1];

%% Calcolo della Raggiungibilità / Controllabilità
% Otteniamo il numero di variabili di stato
n = size(A_long, 1);       

% 1. Calcolo della matrice di Raggiungibilità (Controllabilità)
R = ctrb(A_long, B_long);

% 2. Calcolo del rango della matrice
rango_R = rank(R);    

disp('--- Matrice di Raggiungibilità/Controllabilità R ---');
disp(R);

% 3. Verifica della completa raggiungibilità
% Il sistema è completamente raggiungibile se il rango della matrice R 
% è uguale al numero di variabili di stato n.
if rango_R == n
    disp('Il sistema è COMPLETAMENTE raggiungibile (e controllabile).');
else
    disp('Il sistema NON è completamente raggiungibile (rango incompleto).');
    disp(['Il rango della matrice è ', num2str(rango_R), ' invece di ', num2str(n), '.']);
end

% Calcolo la nuova matrice a ciclo chiuso
A_cl = A_long - B_long * K;

% Calcolo i nuovi poli
poli_chiusi = eig(A_cl);

disp('Autovalori a ciclo chiuso:');
disp(poli_chiusi);

% Verifica automatica in MATLAB
if all(real(poli_chiusi) < 0)
    disp('VERIFICA SUPERATA: Il sistema è stabile. L''autovalore instabile è stato portato a sinistra.');
else
    disp('ATTENZIONE: Il sistema è ancora instabile! Controlla le matrici Q e R.');
end

% =========================================================================
% VISUALIZZAZIONE GRAFICA DEI NUOVI POLI (Ciclo Chiuso)
% =========================================================================

% 1. Creazione del nuovo sistema "controllato" (ciclo chiuso)
% Sostituiamo la matrice A originale con la nuova dinamica A_cl
sys_cl = ss(A_cl, B_long, C_long, D_long);

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