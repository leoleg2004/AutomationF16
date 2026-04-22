% =========================================================================
% ANALISI VALORI SINGOLARI LQR (Valutata all'ingresso dell'impianto)
% =========================================================================
disp('--- Analisi Valori Singolari per Sistema LQR (Max e Min) ---');

% 1. Definizione dell'Anello Aperto L(s) all'ingresso degli attuatori
num_ingressi = size(B_ctrl, 2); 
D_L = zeros(size(K, 1), num_ingressi); 
L_in_sys = ss(A_long, B_ctrl, K, D_L);% prendo lo state space mettendo le matrici 
%A_long,B_ctrl,K che e per LQR u=-k*x e per D una matrice di zeri.

% 2. Calcolo Sensitività S(s) e Complementare T(s)
I_mat = eye(num_ingressi);
S_in_sys = feedback(I_mat, L_in_sys); % S = inv(I + L)
T_in_sys = feedback(L_in_sys, I_mat); % T = L * inv(I + L)

% =========================================================================
% ESTRAZIONE E GRAFICI DEI VALORI SINGOLARI MAX E MIN
% =========================================================================
w = logspace(-3, 3, 500); % Frequenze da 0.001 a 1000 rad/s

% Estrazione matriciale per L(s)
[sv_L, ~] = sigma(L_in_sys, w);
sv_L_dB = 20*log10(sv_L);

% Estrazione matriciale per S(s)
[sv_S, ~] = sigma(S_in_sys, w);
sv_S_dB = 20*log10(sv_S);

% Estrazione matriciale per T(s)
[sv_T, ~] = sigma(T_in_sys, w);
sv_T_dB = 20*log10(sv_T);

% --- GRAFICO 1: ANELLO APERTO L(s) ---
figure('Name', 'LQR: Anello Aperto L(s)', 'Color', 'w', 'Position', [100, 100, 600, 450]);
semilogx(w, sv_L_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   % Sigma Max (Riga 1)
semilogx(w, sv_L_dB(end,:), 'k', 'LineWidth', 2.5);          % Sigma Min (Ultima riga)
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 1.5);        % Riferimento 0 dB (Crossover)
hold off;
grid on;
title('Anello Aperto LQR L(s) all''ingresso (Inviluppo Max/Min)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)');
xlabel('Frequenza (rad/s)');
legend('\sigma_{Max} (Direzione Forte)', '\sigma_{Min} (Direzione Debole)', '0 dB (Crossover)', 'Location', 'best');

% --- GRAFICO 2: SENSITIVITÀ S(s) ---
figure('Name', 'LQR: Sensitività S(s)', 'Color', 'w', 'Position', [150, 150, 600, 450]);
semilogx(w, sv_S_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   % Sigma Max
semilogx(w, sv_S_dB(end,:), 'k', 'LineWidth', 2.5);          % Sigma Min
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 2);          % Limite LQR 0 dB
hold off;
grid on;
title('Sensitività LQR $S(s)$ (Dimostrazione Asintoto e Robustezza)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)');
xlabel('Frequenza (rad/s)');
legend('\sigma_{Max}', '\sigma_{Min}', 'Limite LQR e Asintoto (0 dB)', 'Location', 'best');

% --- GRAFICO 3: COMPLEMENTARE T(s) ---
figure('Name', 'LQR: Complementare T(s)', 'Color', 'w', 'Position', [200, 200, 600, 450]);
semilogx(w, sv_T_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   % Sigma Max
semilogx(w, sv_T_dB(end,:), 'k', 'LineWidth', 2.5);          % Sigma Min
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 2);          % Riferimento 0 dB (Tracking)
hold off;
grid on;
title('Sens. Complementare LQR $T(s)$ (Tracking)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)');
xlabel('Frequenza (rad/s)');
legend('\sigma_{Max}', '\sigma_{Min}', 'Riferimento Tracking (0 dB)', 'Location', 'best');

disp('Grafici dell''inviluppo Max/Min generati con successo!');