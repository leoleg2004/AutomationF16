function plot_lyapunov(P, A_cl)
    % PLOT_LYAPUNOV Disegna il ritratto di fase 2D e la funzione di Lyapunov 3D
    rad2deg = 180 / pi;
    
    % 1. Definizione della griglia spaziale
    w_range = linspace(-25, 25, 50);   % ft/s
    q_range_rad = linspace(-1.5, 1.5, 50); % rad/s
    [W_grid, Q_rad] = meshgrid(w_range, q_range_rad);
    Q_deg = Q_rad * rad2deg; 
    
    % 2. Calcolo della funzione di Lyapunov V(x) sulla griglia
    V_surf = zeros(size(W_grid));
    for i = 1:size(W_grid, 1)
        for j = 1:size(W_grid, 2)
            stato_surf = [0; Q_rad(i,j); 0; W_grid(i,j)];
            V_surf(i,j) = stato_surf' * P * stato_surf;
        end
    end
    
    % 3. Dinamica a ciclo chiuso (Tempo Continuo)
    dxdt = @(t, x) A_cl * x;
    
    % 4. Condizioni iniziali multiple [theta; q; U; W]
    x0_mult = [
        0,   0,    0,   0,    0,   0,    0,   0;      
        1.2, -1.2, 0.5, -0.5, 0.8, -0.8, 0.3, -0.3;   
        0,   0,    0,   0,    0,   0,    0,   0;      
        20,  -20,  15,  -15, -10,  10,   22,  -22     
    ];
    
    % -------------------------------------------------------------------------
    % FIGURA 1: RITRATTO DI FASE 2D CON CURVE DI LIVELLO
    % -------------------------------------------------------------------------
    figure('Name', 'Ritratto di Fase 2D LQR', 'Color', 'w', 'Position', [250, 250, 700, 500]);
    hold on; grid on;
    contour(W_grid, Q_deg, V_surf, 40, 'LineWidth', 1);
    colormap jet; colorbar;
    
    for i = 1:size(x0_mult, 2)
        [~, x_ode] = ode45(dxdt, [0 5], x0_mult(:, i));
        plot(x_ode(:,4), x_ode(:,2) * rad2deg, 'k', 'LineWidth', 1.5);
        plot(x_ode(1,4), x_ode(1,2) * rad2deg, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    end
    
    title('Ritratto di Fase e Curve di Livello $V(x)$ (LQR)', 'Interpreter', 'latex');
    xlabel('W (Velocità Verticale) [ft/s]', 'Interpreter', 'latex');
    ylabel('q (Pitch Rate) [deg/s]', 'Interpreter', 'latex');
    axis tight;
    
    % -------------------------------------------------------------------------
    % FIGURA 2: FUNZIONE DI LYAPUNOV 3D CON TRAIETTORIE
    % -------------------------------------------------------------------------
    figure('Name', 'Funzione di Lyapunov 3D - LQR', 'Color', 'w', 'Position', [300, 300, 800, 600]);
    hold on; grid on;
    surf(W_grid, Q_deg, V_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    colormap jet;
    
    for i = 1:size(x0_mult, 2)
        [~, x_ode] = ode45(dxdt, [0 5], x0_mult(:, i));
        V_traiettoria = sum((x_ode * P) .* x_ode, 2); 
        plot3(x_ode(:,4), x_ode(:,2) * rad2deg, V_traiettoria, 'k-', 'LineWidth', 2);
        plot3(x_ode(1,4), x_ode(1,2) * rad2deg, V_traiettoria(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    end
    
    title('Funzione di Lyapunov $V(x) = x^T P x$ a Ciclo Chiuso', 'Interpreter', 'latex');
    xlabel('$W$ [ft/s]', 'Interpreter', 'latex');
    ylabel('$q$ [deg/s]', 'Interpreter', 'latex');
    zlabel('$V(x)$', 'Interpreter', 'latex');
    view(-35, 35);
end
