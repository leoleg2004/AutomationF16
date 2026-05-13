% =========================================================================
% Tesi Triennale - MPC Longitudinale F-16
% Simulazione con Conversione Continuo-Discreto (c2d) e Radianti
% =========================================================================

%% 1. Definizione del Sistema (TEMPO CONTINUO)
nx = 4; % Stati: [theta,q,U,W]'
nu = 3; % Ingressi: [T, dele, dlef]'
Ts = 0.1; % Tempo di campionamento in secondi


% --- 1B. CONVERSIONE IN TEMPO DISCRETO ---
disp('Conversione del modello da Continuo a Discreto...');
sys_c = ss(A_long, B_ctrl, eye(nx), zeros(nx, nu));
sys_d = c2d(sys_c, Ts, 'zoh'); % Zero-Order Hold
A_long_ds = sys_d.A;
B_ctrl_ds = sys_d.B;

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
xlabel('\Delta V [ft/s]'); ylabel('\Delta \alpha [rad]'); zlabel('q [rad/s]');

[X1, X2, X3] = meshgrid(linspace(-40, 40, 50), ...
                        linspace(-deg2rad(10), deg2rad(10), 50), ...
                        linspace(-deg2rad(60), deg2rad(60), 50));
X1_f = X1(:); X2_f = X2(:); X3_f = X3(:); X4_f = zeros(size(X1_f)); 
Validi = true(size(X1_f));
for j = 1:size(G_inf, 1)
    Valore = G_inf(j,1)*X1_f + G_inf(j,2)*X2_f + G_inf(j,3)*X3_f + G_inf(j,4)*X4_f;
    Validi = Validi & (Valore <= g_inf(j));
end
PX = X1_f(Validi); PY = X2_f(Validi); PZ = X3_f(Validi);
if length(PX) > 4
    K_hull = convhull(PX, PY, PZ);
    trisurf(K_hull, PX, PY, PZ, 'FaceColor', 'c', 'FaceAlpha', 0.15, 'EdgeColor', 'b', 'EdgeAlpha', 0.1);
else
    disp('ATTENZIONE: Griglia 3D troppo stretta o set vuoto.');
end

%% 5. Setup Problema MPC 
N = 10; % Orizzonte predittivo sufficientemente lungo (4 secondi)
n_vars = N*nu + N*nx; 

% ---> RIPRISTINATA STRUTTURA CHIARA CON R_blk E Q_blk <---
R_blk = kron(eye(N), R);
Q_blk = blkdiag(kron(eye(N-1), Q), P);
H = 2 * blkdiag(R_blk, Q_blk);         

f = zeros(n_vars, 1);                  
Aeq_base = zeros(N*nx, n_vars);
for k = 1:N
    Aeq_base((k-1)*nx + 1 : k*nx, (k-1)*nu + 1 : k*nu) = -B_ctrl_ds;
    Aeq_base((k-1)*nx + 1 : k*nx, N*nu + (k-1)*nx + 1 : N*nu + k*nx) = eye(nx);
    if k > 1
        Aeq_base((k-1)*nx + 1 : k*nx, N*nu + (k-2)*nx + 1 : N*nu + (k-1)*nx) = -A_long_ds;
    end
end
lb = -inf(n_vars, 1); ub = inf(n_vars, 1); 
for k = 1:N
    lb((k-1)*nu + 1 : k*nu) = U_min;
    ub((k-1)*nu + 1 : k*nu) = U_max;
end
A_ineq_stat = []; b_ineq_stat = [];
for j = 1:N-1
    A_t = zeros(size(Gx, 1), n_vars);
    A_t(:, N*nu + (j-1)*nx + 1 : N*nu + j*nx) = Gx;
    A_ineq_stat = [A_ineq_stat; A_t];
    b_ineq_stat = [b_ineq_stat; gx];
end
A_t = zeros(size(G_inf, 1), n_vars);
A_t(:, N*nu + (N-1)*nx + 1 : N*nu + N*nx) = G_inf;
A_ineq_stat = [A_ineq_stat; A_t];
b_ineq_stat = [b_ineq_stat; g_inf];

%% 6. Simulazione MPC Completa 
disp('--- Avvio Ottimizzazione e Simulazione MPC ---');

% Ordine: [theta; q; u; w]
x_iniziale = [deg2rad(15);  % theta: 5 gradi convertiti in rad
              deg2rad(12);           % q: velocità angolare nulla
              20;          % u: +10 ft/s di velocità forward
              40];          % w: velocità verticale nulla
t_sim = 20; 
storia_x = zeros(nx, t_sim+1); storia_x(:,1) = x_iniziale;
storia_u = zeros(nu, t_sim);
u_previous = [0;0;0]; 

options = optimoptions('quadprog', 'Display', 'off')
for t = 1:t_sim
    beq = zeros(N*nx, 1);
    beq(1:nx) = A_long_ds * storia_x(:,t); 
    
    A_rate = zeros(nu*2*N, n_vars); b_rate = zeros(nu*2*N, 1);
    for k = 1:N
        idx_u = (k-1)*nu+1:k*nu;
        A_rate((k-1)*2*nu+1:k*2*nu, idx_u) = [eye(nu); -eye(nu)];
        if k == 1
            b_rate((k-1)*2*nu+1:k*2*nu) = [dU_max + u_previous; dU_max - u_previous];
        else
            idx_u_prev = (k-2)*nu+1:(k-1)*nu;
            A_rate((k-1)*2*nu+1:k*2*nu, idx_u_prev) = [-eye(nu); eye(nu)];
            b_rate((k-1)*2*nu+1:k*2*nu) = [dU_max; dU_max];
        end
    end
    [z_opt, ~, exitflag] = quadprog(H, f, [A_ineq_stat; A_rate], [b_ineq_stat; b_rate], Aeq_base, beq, lb, ub, [], options);
    
    if exitflag < 0
        error('Infeasible! Il punto allo step %d è fuori da X_N. Riduci leggermente la severità di x_iniziale.', t);
    end
    
    if t == 1
        X_pred = zeros(nx, N+1);
        X_pred(:, 1) = x_iniziale;
        for k = 1:N
            X_pred(:, k+1) = z_opt(N*nu + (k-1)*nx + 1 : N*nu + k*nx);
        end
        plot3(X_pred(1,:), X_pred(2,:), X_pred(3,:), '-rs', 'LineWidth', 2, 'MarkerFaceColor', 'r');
        plot3(x_iniziale(1), x_iniziale(2), x_iniziale(3), 'k*', 'MarkerSize', 10, 'LineWidth', 2);
        text(x_iniziale(1), x_iniziale(2), x_iniziale(3)+0.1, ' Partenza', 'FontWeight', 'bold');
    end
    
    u_applicata = z_opt(1:nu);
    storia_u(:, t) = u_applicata;
    
    storia_x(:, t+1) = A_long_ds * storia_x(:,t) + B_ctrl_ds * u_applicata;
    u_previous = u_applicata;
end
disp('Ottimizzazione Riuscita! Il modello è matematicamente solido.');

%% 7. Grafici 
figure('Name', 'Risultati MPC: Stati del Velivolo', 'Color', 'w', 'Position', [100 100 900 600]);
subplot(2,2,1); plot(0:t_sim, storia_x(1,:), '-b', 'LineWidth', 1.5); title('\Delta Velocità [ft/s]'); grid on; 
subplot(2,2,2); plot(0:t_sim, storia_x(2,:), '-r', 'LineWidth', 1.5); title('\Delta Angolo d''Attacco [rad]'); grid on; 
subplot(2,2,3); plot(0:t_sim, storia_x(3,:), '-m', 'LineWidth', 1.5); title('Pitch Rate [rad/s]'); grid on; 
subplot(2,2,4); plot(0:t_sim, storia_x(4,:), '-c', 'LineWidth', 1.5); title('\Delta Pitch Angle [rad]'); grid on;

figure('Name', 'Risultati MPC: Sforzo degli Attuatori', 'Color', 'w', 'Position', [150 150 700 800]);
subplot(3,1,1); stairs(0:t_sim-1, storia_u(1,:), '-g', 'LineWidth', 1.5); hold on;
yline(U_max(1), 'k--'); yline(U_min(1), 'k--'); title('\Delta Spinta [lbf]'); grid on; ylim([U_min(1)-2000, U_max(1)+2000]);
subplot(3,1,2); stairs(0:t_sim-1, storia_u(2,:), '-g', 'LineWidth', 1.5); hold on;
yline(U_max(2), 'k--'); yline(U_min(2), 'k--'); title('\Delta Elevatore [rad]'); grid on; ylim([U_min(2)-5, U_max(2)+5]);
subplot(3,1,3); stairs(0:t_sim-1, storia_u(3,:), '-g', 'LineWidth', 1.5); hold on;
yline(U_max(3), 'k--'); yline(U_min(3), 'k--'); title('\Delta Flap [rad]'); grid on; ylim([U_min(3)-5, U_max(3)+5]);