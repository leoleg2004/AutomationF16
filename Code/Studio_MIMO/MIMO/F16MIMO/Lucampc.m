%% 2. Progetto LQR 
% Pesiamo l'errore massimo accettabile: u=20ft/s, w=15ft/s, q=10deg/s, th=10deg
Q = diag([1/20^2, 1/15^2, 1/deg2rad(10)^2, 1/deg2rad(10)^2]); 

% Pesiamo gli attuatori: Spinta=5000lb, Elevatore=25deg, Flap=25deg
% La spinta ha un numero grande, quindi il peso deve essere piccolissimo (1e-7)
R = diag([1/5000^2, 1/25^2, 1/25^2]);     

[K, P, ~] = dlqr(A_long_ds, B_ctrl_ds, Q, R);
A_cl = A_long_ds - B_ctrl_ds*K;
%% 3. Vincoli Fisici (Ampiezza e Rateo)
% Ingressi: [lb, deg, deg]
U_min = [-4000; -25; -12];  
U_max = [10000;  25;  12]; 
Rate_max = [10000; 60; 25]; 
dU_max = Rate_max * Ts; dU_min = -dU_max;

% Stati: theta(rad), q(rad/s), u(ft/s), w(ft/s)
X_max = [ deg2rad(45);  deg2rad(60);  100;  85]; 
X_min = [-deg2rad(45); -deg2rad(60); -100; -85];

Gx = [eye(nx); -eye(nx)]; gx = [X_max; -X_min];
Gu_x = [-K; K]; gu_x = [U_max; -U_min];
G = [Gx; Gu_x]; g = [gx; gu_x];
%% 4. Calcolo e Plot 3D Convergenza di O_inf
G_inf = G; g_inf = g;
max_iter = 100; tol = 1e-6;
disp('--- CALCOLO O_INF ---');
for i = 1:max_iter
    G_next = G * (A_cl^i);
    G_inf_new = [G_inf; G_next];
    g_inf_new = [g_inf; g];
    if norm(G_next, inf) < tol
        fprintf('Convergenza O_inf raggiunta all''iterazione %d\n', i);
        break;
    end
    G_inf = G_inf_new; g_inf = g_inf_new;
end

figure('Name', 'Control Invariant Set 3D', 'Color', 'w', 'Position', [50, 50, 900, 700]);
hold on; grid on; view(3); 
title('Poliedro $\mathcal{X}_f$ in 3D (Fetta $\theta = 0$)', 'Interpreter', 'latex', 'FontSize', 14);

% Assi aggiornati al tuo vettore di stato: [theta, q, U, W]
xlabel('u [ft/s]'); ylabel('w [ft/s]'); zlabel('q [rad/s]');

% Griglia coerente con i tuoi stati (U, W, q)
[X_U, X_W, X_q] = meshgrid(linspace(-50, 50, 40), ... % Range per u (Stato 3)
                           linspace(-50, 50, 40), ... % Range per w (Stato 4)
                           linspace(-deg2rad(40), deg2rad(40), 40)); % Range per q (Stato 2)

X_U_f = X_U(:); X_W_f = X_W(:); X_q_f = X_q(:); 

% Imposta la fetta in modo che combaci con la theta iniziale (15 gradi)
theta_slice = 0; 
X_theta_f = theta_slice * ones(size(X_U_f)); % Fetta corrispondente alla partenza

Validi = true(size(X_U_f));
for j = 1:size(G_inf, 1)
    % Ordine corretto di moltiplicazione: G_inf * [theta; q; u; w]
    Valore = G_inf(j,1)*X_theta_f + G_inf(j,2)*X_q_f + G_inf(j,3)*X_U_f + G_inf(j,4)*X_W_f;
    Validi = Validi & (Valore <= g_inf(j));
end

PX = X_U_f(Validi); PY = X_W_f(Validi); PZ = X_q_f(Validi);

if length(PX) > 4
    K_hull = convhull(PX, PY, PZ);
    trisurf(K_hull, PX, PY, PZ, 'FaceColor', 'c', 'FaceAlpha', 0.15, 'EdgeColor', 'b', 'EdgeAlpha', 0.1);
else
    disp('ATTENZIONE: Nessun punto valido trovato per il plot. Verifica i limiti di meshgrid.');
end