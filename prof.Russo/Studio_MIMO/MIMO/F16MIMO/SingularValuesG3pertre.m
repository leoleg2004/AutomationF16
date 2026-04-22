% =========================================================================
% ANALISI DEI VALORI SINGOLARI E CONDIZIONAMENTO (Norma 1 Indotta)
% =========================================================================
disp('--- Analisi Valori Singolari: G(s) ---');

% MATLAB estrae analiticamente l'amplificazione massima e minima
% per tutti i vettori di ingresso tali che ||u|| = 1.
% sv conterrà 3 righe (essendo un sistema 3x3)
[sv, w] = sigma(G_3x3); 

% Conversione in Decibel (dB)
sv_dB = 20*log10(sv);

% 1. GRAFICO DEL VALORE MASSIMO E MINIMO
figure('Name', 'Valori Singolari (Max e Min)', 'Color', 'w', 'Position', [100, 100, 700, 500]);

% Tracciamo la direzione a massimo guadagno (Norma 1 in ingresso -> Max Uscita)
semilogx(w, sv_dB(1,:), 'b', 'LineWidth', 2.5); 
hold on;
% Tracciamo la direzione a minimo guadagno (Norma 1 in ingresso -> Min Uscita)
semilogx(w, sv_dB(3,:), 'k', 'LineWidth', 2.5); 
hold off;

grid on;
title('Valori Singolari dell''Impianto $G(s)$ (Ingressi ||u||=1)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Amplificazione Indotta (dB)');
xlabel('Frequenza (rad/s)');
legend('\sigma_{Max} (Direzione più reattiva)', '\sigma_{Min} (Direzione più faticosa)', 'Location', 'best');

% 2. NUMERO DI CONDIZIONAMENTO
% Il numero di condizionamento è il rapporto tra Sigma Max e Sigma Min.
% In dB è una semplice sottrazione. Rappresenta la severità del cross-talk.
gamma_dB = sv_dB(1,:) - sv_dB(3,:); 

figure('Name', 'Numero di Condizionamento', 'Color', 'w', 'Position', [150, 150, 700, 500]);
semilogx(w, gamma_dB, 'LineWidth', 2.5, 'Color', 'r');
grid on;
title('Numero di Condizionamento $\gamma(j\omega)$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Condizionamento (dB)');
xlabel('Frequenza (rad/s)');

disp('Grafici generati! Analizza il numero di condizionamento.');



% =========================================================================
% ANALISI DIREZIONALE SVD: INGRESSI E USCITE DELLA MANOVRA MASSIMA
% =========================================================================
disp('--- Analisi Direzionale SVD: Estrazione Matrici U e V ---');

% 1. Definiamo il vettore delle frequenze
w = logspace(-3, 2, 200); 

% 2. Valutiamo la risposta in frequenza dell'impianto (Matrici Complesse)
% freqresp genera un array 3D: (uscite x ingressi x frequenze)
G_jw = freqresp(G_3x3, w); 

% 3. Preallochiamo le matrici per salvare le direzioni massime
% Salveremo solo il modulo (abs) per evitare i salti di segno dell'SVD
Dir_Ingressi = zeros(3, length(w)); % Contributi di Spinta, Eq, Flap
Dir_Uscite   = zeros(3, length(w)); % Contributi di Vt, alpha, q

% 4. Calcolo SVD frequenza per frequenza
for k = 1:length(w)
    % Estraiamo la matrice 3x3 alla frequenza k-esima
    G_k = G_jw(:, :, k); 
    
    % Scomposizione ai Valori Singolari: G = U * S * V^H
    [U, S, V] = svd(G_k); 
    
    % La direzione associata al Sigma Massimo (la manovra più forte) 
    % si trova sempre nella PRIMA colonna di V e di U.
    Dir_Ingressi(:, k) = abs(V(:, 1)); 
    Dir_Uscite(:, k)   = abs(U(:, 1)); 
end

% =========================================================================
% GRAFICI DELLE DIREZIONI
% =========================================================================

% --- Grafico 1: Composizione dell'Ingresso (Chi causa il picco?) ---
figure('Name', 'Direzioni di Ingresso SVD', 'Color', 'w', 'Position', [100, 100, 700, 500]);
semilogx(w, Dir_Ingressi(1,:), 'b', 'LineWidth', 2); hold on; % Spinta
semilogx(w, Dir_Ingressi(2,:), 'r', 'LineWidth', 2);          % Equilibratore
semilogx(w, Dir_Ingressi(3,:), 'g', 'LineWidth', 2); hold off;% Flap
grid on;
title('Composizione della Manovra a Massimo Guadagno (Ingressi $V$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Contributo Relativo (Norma 1)');
xlabel('Frequenza (rad/s)');
legend('\delta_{Thrust} (Spinta)', '\delta_e (Equilibratore)', '\delta_{lef} (Flap)', 'Location', 'best');
% Fissiamo l'asse Y tra 0 e 1, poiché la somma dei quadrati fa sempre 1 (Norma unitaria)
ylim([0, 1.1]); 

% --- Grafico 2: Composizione dell'Uscita (Chi subisce il picco?) ---
figure('Name', 'Direzioni di Uscita SVD', 'Color', 'w', 'Position', [150, 150, 700, 500]);
semilogx(w, Dir_Uscite(1,:), 'b', 'LineWidth', 2); hold on; % Vt
semilogx(w, Dir_Uscite(2,:), 'r', 'LineWidth', 2);          % Alpha
semilogx(w, Dir_Uscite(3,:), 'g', 'LineWidth', 2); hold off;% q
grid on;
title('Effetto della Manovra a Massimo Guadagno (Uscite $U$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Contributo Relativo (Norma 1)');
xlabel('Frequenza (rad/s)');
legend('V_t (Velocità)', '\alpha (Angolo Attacco)', 'q (Pitch Rate)', 'Location', 'best');
ylim([0, 1.1]);

disp('Grafici direzionali generati! Guarda quale colore domina alle varie frequenze.');