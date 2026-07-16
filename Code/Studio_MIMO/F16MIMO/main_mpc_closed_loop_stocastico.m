% =========================================================================
% Tesi Triennale - MPC Longitudinale F-16
% Ing. Leggeri Leonardo
% Analisi di Robustezza: MPC Anello Chiuso in presenza di Vento Stocastico
% =========================================================================



% 1. Eseguiamo il setup nominale originale per generare le strutture
disp('Inizializzazione del problema MPC nominale...');
MPC; % Esegue il tuo file MPC.m, popolando il workspace

% Al termine dello script MPC, salviamo le traiettorie nominali perfette
storia_x_nom = storia_x;
storia_u_nom = storia_u;

% 2. Setup della Tempesta (Matrice di Diffusione G)
disp('Configurazione della tempesta di vento (Moto Browniano)...');
Bwind = B_long(:, end-1:end); 
sigma_vento_1 = 0.50; 
sigma_vento_2 = 0.50; 
G = Bwind * diag([sigma_vento_1, sigma_vento_2]);

% 3. Simulazione Anello Chiuso Stocastico
disp('--- Avvio Simulazione MPC ANELLO CHIUSO con Vento ---');
% Lanciamo la nuova funzione che calcola il quadprog ad ogni step con lo stato perturbato
[storia_x_stoc, storia_u_stoc, storia_costo_stoc] = simula_mpc_stocastico(mpc_prob, x_iniziale, t_sim, A_long_ds, B_ctrl_ds, dU_max, G, Ts);
disp('Simulazione Completata!');

% 4. Plot dei risultati Comparativi
tabs = crea_dashboard_mpc(); % Recupera i tab generati da MPC.m

t_plot = (0:t_sim) * Ts;
t_plot_u = (0:t_sim-1) * Ts;

% Impostiamo l'interprete LaTeX per avere testi e formule eleganti
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');

% PLOT STATI
t_layout_stoc_x = tiledlayout(tabs.stoc_stati, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nomi_stati = {'Angolo di Beccheggio ($\theta$) [rad]', 'Velocit\`a di Beccheggio ($q$) [rad/s]', 'Velocit\`a asse X ($U$) [ft/s]', 'Velocit\`a asse Z ($W$) [ft/s]'};
for i=1:4
    ax_stati = nexttile(t_layout_stoc_x);
    stairs(ax_stati, t_plot, storia_x_nom(i,:), '--k', 'LineWidth', 1.5); hold(ax_stati, 'on');
    stairs(ax_stati, t_plot, storia_x_stoc(i,:), 'b', 'LineWidth', 1.2);
    title(ax_stati, nomi_stati{i}); xlabel(ax_stati, 'Tempo [s]'); 
    legend(ax_stati, 'Nominale (Senza Vento)', 'Anello Chiuso (Col Vento)', 'Location', 'Best');
    grid(ax_stati, 'on');
end

% PLOT INGRESSI ATTUATORI (Il vero lavoro del feedback!)
t_layout_stoc_u = tiledlayout(tabs.stoc_attuatori, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nomi_ingressi = {'Motore / Thrust ($T$) [lbf]', 'Elevatore ($\delta_e$) [deg]', 'Flaperon ($\delta_f$) [deg]'};

% Copia temporanea per conversioni in gradi
u_nom_plot = storia_u_nom;
u_stoc_plot = storia_u_stoc;
u_nom_plot(2:3, :) = u_nom_plot(2:3, :) * (180/pi);
u_stoc_plot(2:3, :) = u_stoc_plot(2:3, :) * (180/pi);

for i=1:3
    ax_u = nexttile(t_layout_stoc_u);
    stairs(ax_u, t_plot_u, u_nom_plot(i,:), '--k', 'LineWidth', 1.5); hold(ax_u, 'on');
    stairs(ax_u, t_plot_u, u_stoc_plot(i,:), 'r', 'LineWidth', 1.2);
    title(ax_u, ['Azione Attuatore: ', nomi_ingressi{i}]); xlabel(ax_u, 'Tempo [s]'); ylabel(ax_u, 'Comando');
    legend(ax_u, 'Azione Nominale Programmata', 'Azione Correttiva (Feedback Reale)', 'Location', 'Best');
    grid(ax_u, 'on');
end

%% 5. Overlay della Traiettoria sul Control Invariant Set (CIS)
disp('Generazione di un nuovo grafico 3D del CIS per la traiettoria stocastica...');
% Cancelliamo il contenuto del tab CIS esistente (creato da MPC.m) e lo ridisegniamo
delete(allchild(tabs.cis_3d));
% In questo modo evitiamo completamente i bug del motore grafico OpenGL di MATLAB 
% ("Could not find node in peer tree") che si verificano cercando di modificare 
% figure complesse rimaste in background.
plot_cis(G_inf, g_inf, x_ref, mpc_prob, A_long_ds, dU_max, tabs.cis_3d);

% Recuperiamo l'oggetto axes (il grafico vero e proprio) dentro la scheda CIS
ax_cis = findobj(tabs.cis_3d, 'Type', 'axes');

% La funzione plot_cis lascia la figura attiva con "hold on"
% 1. Tracciamo la traiettoria Nominale (Blu) per confronto
plot3(ax_cis, storia_x_nom(3,:), storia_x_nom(4,:), storia_x_nom(2,:), '-b', 'LineWidth', 2.5, 'DisplayName', 'Traiettoria Nominale');

% 2. Tracciamo la traiettoria perturbata (Rossa)
plot3(ax_cis, storia_x_stoc(3,:), storia_x_stoc(4,:), storia_x_stoc(2,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'Traiettoria Stocastica (Closed-Loop)');

% 3. Punto di Arrivo finale della simulazione perturbata
plot3(ax_cis, storia_x_stoc(3,end), storia_x_stoc(4,end), storia_x_stoc(2,end), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', 'Arrivo Stocastico');

% Forza l'aggiornamento grafico sicuro
drawnow; 
disp('Traiettoria stocastica tracciata con successo sul NUOVO plot del CIS 3D.');

% Ripristiniamo l'interprete di default di MATLAB per non alterare script futuri
set(groot, 'defaultTextInterpreter', 'tex');
set(groot, 'defaultLegendInterpreter', 'tex');
set(groot, 'defaultAxesTickLabelInterpreter', 'tex');
