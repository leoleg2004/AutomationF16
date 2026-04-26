% =========================================================================
% ANALISI VALORI SINGOLARI LQR (Valutata all'ingresso dell'impianto)
% =========================================================================
disp('--- Analisi Valori Singolari per Sistema LQR (Max e Min) ---');

% 1. Definizione dell'Anello Aperto L(s) all'ingresso degli attuatori
num_ingressi = size(B_ctrl, 2); 
D_L = zeros(size(K, 1), num_ingressi); 
L_in_sys = ss(A_long, B_ctrl, K, D_L); % L(s) = K * inv(sI - A) * B

% 2. Calcolo Sensitività S(s) e Complementare T(s)
I_mat = eye(num_ingressi);
S_in_sys = feedback(I_mat, L_in_sys); % S = inv(I + L)
T_in_sys = feedback(L_in_sys, I_mat); % T = L * inv(I + L)

% =========================================================================
% ESTRAZIONE E GRAFICI DEI VALORI SINGOLARI MAX E MIN
% =========================================================================
w = logspace(-3, 3, 500); % Frequenze da 0.001 a 1000 rad/s

% Estrazione matriciale in dB
[sv_L, ~] = sigma(L_in_sys, w);
sv_L_dB = 20*log10(sv_L);
[sv_S, ~] = sigma(S_in_sys, w);
sv_S_dB = 20*log10(sv_S);
[sv_T, ~] = sigma(T_in_sys, w); 
sv_T_dB = 20*log10(sv_T);

% --- GRAFICO 1: ANELLO APERTO L(s) ---
figure('Name', 'LQR: Anello Aperto L(s)', 'Color', 'w', 'Position', [100, 100, 600, 450]);
semilogx(w, sv_L_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   
semilogx(w, sv_L_dB(end,:), 'k', 'LineWidth', 2.5);          
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 1.5);        
hold off; grid on;
title('Anello Aperto LQR L(s) all''ingresso (Inviluppo Max/Min)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)'); xlabel('Frequenza (rad/s)');
legend('\sigma_{Max} (Direzione Forte)', '\sigma_{Min} (Direzione Debole)', '0 dB (Crossover)', 'Location', 'best');

% --- GRAFICO 2: SENSITIVITÀ S(s) ---
figure('Name', 'LQR: Sensitività S(s)', 'Color', 'w', 'Position', [150, 150, 600, 450]);
semilogx(w, sv_S_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   
semilogx(w, sv_S_dB(end,:), 'k', 'LineWidth', 2.5);          
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 2);          
hold off; grid on;
title('Sensitività LQR S(s) (Dimostrazione Asintoto e Robustezza)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)'); xlabel('Frequenza (rad/s)');
legend('\sigma_{Max}', '\sigma_{Min}', 'Limite LQR e Asintoto (0 dB)', 'Location', 'best');

% --- GRAFICO 3: COMPLEMENTARE T(s) ---
figure('Name', 'LQR: Complementare T(s)', 'Color', 'w', 'Position', [200, 200, 600, 450]);
semilogx(w, sv_T_dB(1,:), 'b', 'LineWidth', 2.5); hold on;   
semilogx(w, sv_T_dB(end,:), 'k', 'LineWidth', 2.5);          
semilogx(w, zeros(size(w)), 'r--', 'LineWidth', 2);          
hold off; grid on;
title('Sens. Complementare LQR $T(s)$ (Tracking)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valori Singolari (dB)'); xlabel('Frequenza (rad/s)');
legend('\sigma_{Max}', '\sigma_{Min}', 'Riferimento Tracking (0 dB)', 'Location', 'best');

% =========================================================================
% VERDETTO AUTOMATICO DEI LIMITI A BASSA FREQUENZA
% =========================================================================
disp('===============================================================');
disp('   VERIFICA ASINTOTICA A BASSA FREQUENZA (w = 0.001 rad/s)     ');
disp('===============================================================');

% Estraiamo i valori alla primissima frequenza w(1) = 10^-3
S_max_lf = sv_S_dB(1, 1);
S_min_lf = sv_S_dB(end, 1);
T_max_lf = sv_T_dB(1, 1);
T_min_lf = sv_T_dB(end, 1);

fprintf('--> SENSITIVITA'' S(s) [Obiettivo teorico: S tende a 0, ovvero -inf dB]\n');
fprintf('    Valore Massimo (Direzione debole): %.2f dB\n', S_max_lf);
fprintf('    Valore Minimo  (Direzione forte):  %.2f dB\n', S_min_lf);
if S_max_lf > -5
    disp('    [!] NOTA: La direzione peggiore NON annulla l''errore/disturbo (S = 0 dB).');
    disp('        Questo è normale: l''LQR è puramente proporzionale e non ha un integratore!');
else
    disp('    [V] SUCCESSO: S(s) tende a 0 (valori ampiamente negativi in dB).');
end

fprintf('\n--> COMPLEMENTARE T(s) [Obiettivo teorico: T tende a 1, ovvero 0 dB]\n');
fprintf('    Valore Massimo (Direzione forte):  %.2f dB\n', T_max_lf);
fprintf('    Valore Minimo  (Direzione debole): %.2f dB\n', T_min_lf);
if abs(T_max_lf) < 2
    disp('    [V] SUCCESSO: Il tracking nella direzione principale è eccellente (T = 0 dB -> Guadagno 1).');
else
    disp('    [!] NOTA: Il tracking presenta errore a regime elevato anche sulla direzione migliore.');
end
disp('===============================================================');