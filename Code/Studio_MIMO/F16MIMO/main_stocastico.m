% MAIN_STOCASTICO
% Script principale per testare il modello stocastico dell'F-16

close all;

addpath('modello_stocastico');

%% 1. Acquisizione Matrici dal Workspace
A = A_long; 
Bwind = B_long(:, end-1:end); 

%% 2. Acquisizione degli Ingressi dell'MPC (dal workspace)
%bisogna far partire prima lo scirpt mpc.m e andare nalla cartella
%dati_Mat e usare dati_da_usare
t_final = (size(storia_u, 2) - 1) * Ts; 
if t_final <= 0
    t_final = 10; 
end
dt = 0.01; 
t = 0:dt:t_final;

t_mpc = (0:(size(storia_u,2)-1)) * Ts; 
U_matrix = interp1(t_mpc, storia_u', t, 'previous', 'extrap')';

%% 3. Definizione della Matrice di Diffusione G (Rumore / Vento)
sigma_vento_1 = 0.5; 
sigma_vento_2 = 0.5; 

G = Bwind * diag([sigma_vento_1, sigma_vento_2]); 

%% 4. Condizione Iniziale
x0 = x_iniziale; 

%% 5. Simulazione Stocastica
disp('Avvio simulazione stocastica (Eulero-Maruyama) con ingressi MPC...');
% Ora preleviamo anche V2_ito e la tassa_accumulata (calcolati con il Lemma di Itô)
[t, X, V2_ito, tassa_accumulata] = simulate_stochastic_F16(A, B_ctrl, G, U_matrix, x0, t_final, dt);

%% 6. Plot dei Risultati: CONFRONTO Nominale vs Stocastico
t_mpc_x = (0:(size(storia_x,2)-1)) * Ts;

% Impostiamo l'interprete LaTeX per avere testi e formule eleganti
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');

figure('Name', 'Confronto F-16: Nominale (MPC) vs Stocastico (MPC + Vento)', 'NumberTitle', 'off');

% Theta
subplot(2,2,1);
plot(t_mpc_x, storia_x(1,:), '--k', 'LineWidth', 1.5); hold on;
plot(t, X(1,:), 'b', 'LineWidth', 1.2);
title('Angolo di Beccheggio ($\theta$)'); xlabel('Tempo [s]'); ylabel('[rad]'); 
legend('Nominale', 'Reale (Vento)', 'Location', 'Best'); grid on;

% q
subplot(2,2,2);
plot(t_mpc_x, storia_x(2,:), '--k', 'LineWidth', 1.5); hold on;
plot(t, X(2,:), 'r', 'LineWidth', 1.2);
title('Velocit\`a di Beccheggio ($q$)'); xlabel('Tempo [s]'); ylabel('[rad/s]'); 
legend('Nominale', 'Reale', 'Location', 'Best'); grid on;

% U
subplot(2,2,3);
plot(t_mpc_x, storia_x(3,:), '--k', 'LineWidth', 1.5); hold on;
plot(t, X(3,:), 'g', 'LineWidth', 1.2);
title('Velocit\`a asse X ($U$)'); xlabel('Tempo [s]'); ylabel('[m/s]'); 
legend('Nominale', 'Reale', 'Location', 'Best'); grid on;

% W
subplot(2,2,4);
plot(t_mpc_x, storia_x(4,:), '--k', 'LineWidth', 1.5); hold on;
plot(t, X(4,:), 'm', 'LineWidth', 1.2);
title('Velocit\`a asse Z ($W$)'); xlabel('Tempo [s]'); ylabel('[m/s]'); 
legend('Nominale', 'Reale', 'Location', 'Best'); grid on;

%% 7. Plot dell'Energia Cinetica: Validazione Lemma di Itô!
% Calcoliamo V^2 nominale (senza vento, dallo script MPC)
V2_nominale = storia_x(3,:).^2 + storia_x(4,:).^2;

% Calcoliamo V^2 in modo "classico/algebrico" prendendo i risultati stocastici
V2_algebraico = X(3,:).^2 + X(4,:).^2;

figure('Name', 'Validazione Lemma di Itô sull''Energia Cinetica', 'NumberTitle', 'off');
plot(t_mpc_x, V2_nominale, '--k', 'LineWidth', 1.5); hold on;
plot(t, V2_algebraico, 'b', 'LineWidth', 2); 
plot(t, V2_ito, '--r', 'LineWidth', 2);
title('Evoluzione Stocastica di $V^2$ ($\propto$ Energia Cinetica)');
xlabel('Tempo [s]'); ylabel('$V^2$ [(m/s)$^2$]');
legend('Nominale senza vento (MPC)', ...
       'Reale con vento (Calcolo Standard $U^2 + W^2$)', ...
       'Reale con vento (Integrazione Numerica It\^o)', ...
       'Location', 'Best');
grid on;

%% 8. Plot della Tassa di Itô Accumulata
figure('Name', 'Avanzamento della Tassa di Itô nel Tempo', 'NumberTitle', 'off');
plot(t, tassa_accumulata, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); 
title('Drift Energetico Accumulato (Tassa di It\^o)');
xlabel('Tempo [s]'); ylabel('Energia extra introdotta [$(m/s)^2$]');
grid on;

%% 9. Plot degli Ingressi (Attuatori)
% Poiché stiamo simulando ad Anello Aperto, gli ingressi del modello 
% stocastico sono esattamente gli stessi pre-calcolati dall'MPC nominale.
figure('Name', 'Confronto Ingressi Attuatori', 'NumberTitle', 'off');
num_ingressi = size(U_matrix, 1);
for i = 1:num_ingressi
    subplot(num_ingressi, 1, i);
    % Plot del segnale nominale originario
    stairs(t_mpc, storia_u(i,:), '--k', 'LineWidth', 1.5); hold on;
    % Plot del segnale campionato ad alta frequenza fornito al modello stocastico
    plot(t, U_matrix(i,:), 'b', 'LineWidth', 1.2);
    title(['Ingresso Attuatore ', num2str(i)]);
    xlabel('Tempo [s]'); ylabel('Comando');
    legend('Calcolato da MPC (Nominale)', 'Inviato all''aereo nel vento', 'Location', 'Best');
    grid on;
end

% Ripristiniamo l'interprete di default di MATLAB per non alterare script futuri
set(groot, 'defaultTextInterpreter', 'tex');
set(groot, 'defaultLegendInterpreter', 'tex');
set(groot, 'defaultAxesTickLabelInterpreter', 'tex');

disp('Simulazione completata con successo!');
