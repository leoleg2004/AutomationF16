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

function plot_cis(G_inf, g_inf, x_ref, mpc_prob, A_long_ds, dU_max, parent_tab)
    if nargin < 3
        x_ref = zeros(4,1);
    end
    if nargin < 7
        fig1 = figure('Name', 'Control Invariant Set 3D', 'Color', 'w', 'Position', [50, 50, 900, 700]);
        parent_tab = fig1;
    end
    
    % PLOT_CIS Disegna in 3D il Control Invariant Set
    ax = axes('Parent', parent_tab);
    hold(ax, 'on'); grid(ax, 'on'); view(ax, 3); 
    title(ax, ['Poliedro $\mathcal{X}_f$ in 3D (Fetta $\theta = ', num2str(x_ref(1)), '$)'], 'Interpreter', 'latex', 'FontSize', 14);
    
    % Assi aggiornati al tuo vettore di stato: [theta, q, U, W]
    xlabel('u [m/s]'); ylabel('w [m/s]'); zlabel('q [rad/s]');
    
    % Griglia ad alta risoluzione centrata vicino all'origine per trovare il CIS
    [X_U, X_W, X_q] = meshgrid(linspace(-30, 30, 40), ... % Range per u (Stato 3)
                               linspace(-30, 30, 40), ... % Range per w (Stato 4)
                               linspace(-40 * pi/180, 40 * pi/180, 40)); % Range per q (Stato 2)
    
    X_U_f = X_U(:); X_W_f = X_W(:); X_q_f = X_q(:); 
    
    % Imposta la fetta in modo che combaci con la theta del target
    theta_slice = x_ref(1); 
    X_theta_f = theta_slice * ones(size(X_U_f)); % Fetta corrispondente al target
    
    Validi_inf = true(size(X_U_f));
    for j = 1:size(G_inf, 1)
        Valore = G_inf(j,1)*X_theta_f + G_inf(j,2)*X_q_f + G_inf(j,3)*X_U_f + G_inf(j,4)*X_W_f;
        Validi_inf = Validi_inf & (Valore <= g_inf(j));
    end
    
    PX_inf = X_U_f(Validi_inf); PY_inf = X_W_f(Validi_inf); PZ_inf = X_q_f(Validi_inf);
    
    if length(PX_inf) > 4
        K_hull_inf = convhull(PX_inf, PY_inf, PZ_inf);
        h_inf = trisurf(K_hull_inf, PX_inf, PY_inf, PZ_inf, 'Parent', ax, 'FaceColor', 'c', 'FaceAlpha', 0.8, 'EdgeColor', 'b', 'EdgeAlpha', 0.3);
        leg_handles = h_inf;
        leg_names = {'Control Invariant Set $\mathcal{O}_\infty$'};
    else
        disp('ATTENZIONE: Nessun punto valido trovato per il plot di O_inf sulla fetta scelta.');
        leg_handles = [];
        leg_names = {};
    end

    % --- NOVITÀ: Calcolo e Plot N-Step Feasible Set ---
    if nargin >= 6
        disp('Calcolo del Set Feasible N-Step X_N in corso (esplorazione vincoli)...');
        nx = mpc_prob.nx;
        nu = mpc_prob.nu;
        N = mpc_prob.N;
        n_vars = mpc_prob.n_vars;
        
        options = optimoptions('linprog', 'Display', 'off');
        A_rate = zeros(nu*2*N, n_vars); 
        b_rate_base = zeros(nu*2*N, 1);
        for k = 1:N
            idx_u = (k-1)*nu+1:k*nu;
            A_rate((k-1)*2*nu+1:k*2*nu, idx_u) = [eye(nu); -eye(nu)];
            if k > 1
                idx_u_prev = (k-2)*nu+1:(k-1)*nu;
                A_rate((k-1)*2*nu+1:k*2*nu, idx_u_prev) = [-eye(nu); eye(nu)];
                b_rate_base((k-1)*2*nu+1:k*2*nu) = [dU_max; dU_max];
            end
        end
        A_ineq_tot = [mpc_prob.A_ineq_stat; A_rate];
        
        u_previous = [0;0;0];
        b_rate = b_rate_base;
        b_rate(1:2*nu) = [dU_max + u_previous; dU_max - u_previous];
        b_ineq_tot = [mpc_prob.b_ineq_stat; b_rate];

        % Griglia a bassa risoluzione dedicata SOLO all'N-Step Set per velocizzare linprog
        [X_U_N, X_W_N, X_q_N] = meshgrid(linspace(-40, 40, 15), ... 
                                         linspace(-40, 40, 15), ... 
                                         linspace(-40 * pi/180, 40 * pi/180, 15));
        
        X_U_N_f = X_U_N(:); X_W_N_f = X_W_N(:); X_q_N_f = X_q_N(:); 
        f_lin = zeros(n_vars, 1);
        Validi_N = false(size(X_U_N_f));
        
        for i = 1:length(X_U_N_f)
            % Check rapido se è già in O_inf per saltare linprog
            x0_test = [theta_slice; X_q_N_f(i); X_U_N_f(i); X_W_N_f(i)];
            
            is_in_O_inf = true;
            for j = 1:size(G_inf, 1)
                if G_inf(j,:) * x0_test > g_inf(j)
                    is_in_O_inf = false;
                    break;
                end
            end
            
            if is_in_O_inf
                Validi_N(i) = true;
                continue;
            end
            
            % Se non è in O_inf, testa con linprog
            beq = zeros(N*nx, 1);
            beq(1:nx) = A_long_ds * x0_test; 
            
            [~, ~, exitflag] = linprog(f_lin, A_ineq_tot, b_ineq_tot, mpc_prob.Aeq_base, beq, mpc_prob.lb, mpc_prob.ub, options);
            if exitflag >= 0 || exitflag == -3
                Validi_N(i) = true;
            end
        end
        
        PX_N = X_U_N_f(Validi_N); PY_N = X_W_N_f(Validi_N); PZ_N = X_q_N_f(Validi_N);
        if length(PX_N) > 4
            K_hull_N = convhull(PX_N, PY_N, PZ_N);
            h_N = trisurf(K_hull_N, PX_N, PY_N, PZ_N, 'Parent', ax, 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'y', 'EdgeAlpha', 0.1);
            leg_handles(end+1) = h_N;
            leg_names{end+1} = sprintf('Feasible N-Step Set $\\mathcal{X}_{%d}$', N);
            title(ax, ['Poliedri $\mathcal{O}_\infty$ e $\mathcal{X}_{', num2str(N), '}$ in 3D (Fetta $\theta = ', num2str(theta_slice), '$)'], 'Interpreter', 'latex', 'FontSize', 14);
        end
    end
    
    if ~isempty(leg_handles)
        legend(leg_handles, leg_names, 'Location', 'best', 'Interpreter', 'latex');
    end
end
