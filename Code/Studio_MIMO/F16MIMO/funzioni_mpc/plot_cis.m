function plot_cis(G_inf, g_inf, x_ref)
    if nargin < 3
        x_ref = zeros(4,1);
    end
    
    % PLOT_CIS Disegna in 3D il Control Invariant Set
    figure('Name', 'Control Invariant Set 3D', 'Color', 'w', 'Position', [50, 50, 900, 700]);
    hold on; grid on; view(3); 
    title(['Poliedro $\mathcal{X}_f$ in 3D (Fetta $\theta = ', num2str(x_ref(1)), '$)'], 'Interpreter', 'latex', 'FontSize', 14);
    
    % Assi aggiornati al tuo vettore di stato: [theta, q, U, W]
    xlabel('u [ft/s]'); ylabel('w [ft/s]'); zlabel('q [rad/s]');
    
    % Griglia coerente con i tuoi stati (U, W, q)
    [X_U, X_W, X_q] = meshgrid(linspace(-50, 50, 40), ... % Range per u (Stato 3)
                               linspace(-50, 50, 40), ... % Range per w (Stato 4)
                               linspace(-deg2rad(40), deg2rad(40), 40)); % Range per q (Stato 2)
    
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
        trisurf(K_hull_inf, PX_inf, PY_inf, PZ_inf, 'FaceColor', 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'b', 'EdgeAlpha', 0.3);
    else
        disp('ATTENZIONE: Nessun punto valido trovato per il plot di O_inf sulla fetta scelta.');
    end
    
    % Aggiungi il Target al Plot
    plot3(x_ref(3), x_ref(4), x_ref(2), 'kX', 'MarkerSize', 12, 'LineWidth', 3);
    text(x_ref(3), x_ref(4), x_ref(2)+0.05, ' Target', 'FontWeight', 'bold', 'FontSize', 12);
    
    legend({'Control Invariant Set $\mathcal{O}_\infty$', 'Target di Riferimento'}, 'Location', 'best', 'Interpreter', 'latex');
end
