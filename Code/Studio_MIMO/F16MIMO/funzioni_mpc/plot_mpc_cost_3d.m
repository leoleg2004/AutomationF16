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

function plot_mpc_cost_3d(mpc_prob, A_long_ds, dU_max, storia_x, storia_costo, Ts, parent_tab)
    % =========================================================================
    % PLOT_MPC_COST_3D - Visualizza il Funzionale di Costo Ottimo J*(x) dell'MPC
    % =========================================================================

    if nargin < 6
        Ts = 0.1;
    end
    if nargin < 7
        fig1 = figure('Name', sprintf('Costo MPC 3D (Ts = %g s)', Ts), 'Color', 'w', 'Position', [150 150 850 650]);
        parent_tab = fig1;
    end
    rad2deg = 180 / pi;

    w_lim = 40;  % ft/s
    q_lim = 40;  % deg/s

    % Usiamo una griglia 20x20 per un calcolo rapido
    [W_grid, Q_deg] = meshgrid(linspace(-w_lim, w_lim, 20), linspace(-q_lim, q_lim, 20));
    Q_rad = Q_deg / rad2deg; 

    nx = mpc_prob.nx;
    nu = mpc_prob.nu;
    N = mpc_prob.N;
    n_vars = mpc_prob.n_vars;

    % Recuperiamo P_ds approssimato (o potresti passarlo da MPC.m)
    [~, P_ds, ~] = dlqr(A_long_ds, mpc_prob.Aeq_base(1:nx, 1:nu)*(-1), blkdiag(10,10,1,1), eye(nu)); 
    
    V_surf = zeros(size(W_grid));
    for i = 1:size(W_grid, 1)
        for j = 1:size(W_grid, 2)
            stato_surf = [0; Q_rad(i,j); 0; W_grid(i,j)];
            V_surf(i,j) = stato_surf' * P_ds * stato_surf;
        end
    end

    %% --- FIGURA 3D ---
    ax = axes('Parent', parent_tab);
    hold(ax, 'on'); grid(ax, 'on');
    
    h_surf = surf(ax, W_grid, Q_deg, V_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.65);
    colormap(ax, jet);
    cb = colorbar(ax);
    ylabel(cb, 'Costo Ottimo MPC $J^*(x_k)$', 'Interpreter', 'latex', 'FontSize', 12);
    
    max_v_traj = max(storia_costo);
    if isempty(max_v_traj) || isnan(max_v_traj)
        max_v_traj = 1;
    end

    % Traiettoria
    N_steps = length(storia_costo);
    t_discrete = 1:N_steps;
    t_fine = linspace(1, N_steps, N_steps * 10);
    
    W_smooth = pchip(t_discrete, storia_x(4,1:N_steps), t_fine);
    Q_smooth = pchip(t_discrete, storia_x(2,1:N_steps) * rad2deg, t_fine);
    V_smooth = pchip(t_discrete, storia_costo, t_fine);
    
    h_line = plot3(ax, W_smooth, Q_smooth, V_smooth, '-r', 'LineWidth', 2);

    % Punti discreti veri
    plot3(ax, storia_x(4,1:N_steps), storia_x(2,1:N_steps) * rad2deg, storia_costo, 'o', ...
          'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'w', 'MarkerSize', 4);
          
    % Start
    h_start = plot3(ax, storia_x(4,1), storia_x(2,1) * rad2deg, storia_costo(1), 'o', ...
                    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
    
    % End
    h_end = plot3(ax, storia_x(4,N_steps), storia_x(2,N_steps) * rad2deg, storia_costo(N_steps), 's', ...
          'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerSize', 8);

    title(ax, sprintf('\\textbf{Funzionale di Costo MPC $J^*(x_k)$ a Tempo Discreto ($T_s = %g$ s)}', Ts), 'Interpreter', 'latex', 'FontSize', 16);
    xlabel(ax, 'Velocit\`a Verticale $w$ [ft/s]', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel(ax, 'Pitch Rate $q$ [deg/s]', 'Interpreter', 'latex', 'FontSize', 12);
    zlabel(ax, 'Costo $J^*(x_k)$', 'Interpreter', 'latex', 'FontSize', 12);
    
    legend(ax, [h_surf, h_line, h_start, h_end], ...
           {'Superficie $J^*(x_k)$', 'Evoluzione di Stato', 'Partenza ($k=0$)', 'Arrivo'}, ...
           'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
    
    zlim(ax, [0, max(max(V_surf(:)), max_v_traj) * 1.2]); 
    xlim(ax, [-w_lim, w_lim]);
    ylim(ax, [-q_lim, q_lim]);
    view(ax, -35, 30);
end
