% =========================================================================
% Copyright (c) 2026 Ing. Leggeri Leonardo
% Tutti i diritti riservati.
%
% ATTENZIONE: Questo software e il relativo codice sorgente sono di proprietà 
% esclusiva dell'Ing. Leggeri Leonardo. È severamente vietata la copia, 
% la distribuzione, la modifica o la vendita a terzi senza l'esplicito 
% consenso scritto dell'autore. La distribuzione o la vendita non autorizzata 
% costituisce reato ed è perseguibile penalmente secondo le leggi vigenti.
% =========================================================================

%funzione per la creazione dei grafici dopo esecuzione script MPC.m
function plot_risultati(t_sim, storia_x, storia_u, U_min, U_max, x_ref, u_ref, tab_stati, tab_attuatori)
    if nargin < 6
        x_ref = zeros(4,1);
        u_ref = zeros(3,1);
    end
    if nargin < 8
        fig1 = figure('Name', 'Risultati MPC: Stati del Velivolo', 'Color', 'w', 'Position', [100 100 900 600]);
        tab_stati = fig1;
        fig2 = figure('Name', 'Risultati MPC: Sforzo degli Attuatori', 'Color', 'w', 'Position', [150 150 700 800]);
        tab_attuatori = fig2;
    end

    % PLOT_RISULTATI Disegna i grafici dell'evoluzione di stati e attuatori
    t_layout_x = tiledlayout(tab_stati, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    nexttile(t_layout_x); plot(0:t_sim, storia_x(1,:), '-b', 'LineWidth', 1.5); hold on; yline(x_ref(1), 'r--', 'LineWidth', 1.2); title('Pitch Angle (\theta) [rad]'); grid on; 
    nexttile(t_layout_x); plot(0:t_sim, storia_x(2,:), '-r', 'LineWidth', 1.5); hold on; yline(x_ref(2), 'r--', 'LineWidth', 1.2); title('Pitch Rate (q) [rad/s]'); grid on; 
    nexttile(t_layout_x); plot(0:t_sim, storia_x(3,:), '-m', 'LineWidth', 1.5); hold on; yline(x_ref(3), 'r--', 'LineWidth', 1.2); title('Velocità Forward (u) [m/s]'); grid on; 
    nexttile(t_layout_x); plot(0:t_sim, storia_x(4,:), '-c', 'LineWidth', 1.5); hold on; yline(x_ref(4), 'r--', 'LineWidth', 1.2); title('Velocità Verticale (w) [m/s]'); grid on;
    lgd = legend({'Traiettoria', 'Target (x_{ref})'}, 'Orientation', 'horizontal');
    lgd.Layout.Tile = 'north';

    t_layout_u = tiledlayout(tab_attuatori, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    nexttile(t_layout_u); stairs(0:t_sim-1, storia_u(1,:), '-g', 'LineWidth', 1.5); hold on;
    yline(U_max(1), 'k--'); yline(U_min(1), 'k--'); yline(u_ref(1), 'r--', 'LineWidth', 1.2); title(' Spinta [N]'); grid on;
    
    nexttile(t_layout_u); stairs(0:t_sim-1, storia_u(2,:) * 180 / pi, '-g', 'LineWidth', 1.5); hold on;
    yline(U_max(2) * 180 / pi, 'k--'); yline(U_min(2) * 180 / pi, 'k--'); yline(u_ref(2) * 180 / pi, 'r--', 'LineWidth', 1.2);
    title(' Elevatore [deg]'); grid on;
    
    nexttile(t_layout_u); stairs(0:t_sim-1, storia_u(3,:) * 180 / pi, '-g', 'LineWidth', 1.5); hold on;
    yline(U_max(3) * 180 / pi, 'k--'); yline(U_min(3) * 180 / pi, 'k--'); yline(u_ref(3) * 180 / pi, 'r--', 'LineWidth', 1.2);
    title(' Flap [deg]'); grid on;
end
