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

% =========================================================================
% Ritratto di Fase F-16: MOTO FUGOIDE (Phugoid)
% Variabili isolate: theta (deg) vs u (ft/s)
% Ing: Leggeri Leonardo
% =========================================================================


set(0,'DefaultLineLineWidth',1.5);
set(0,'DefaultAxesFontSize',14);
set(0,'DefaulttextInterpreter','latex');

% Verifica matrici
if ~exist('A_long','var') || ~exist('B_ctrl','var')
    error('Le matrici A_long e B_ctrl non sono presenti.');
end

%% 1. Progetto LQR
Q = diag([1/deg2rad(10)^2, 1/deg2rad(20)^2, 1/40^2, 1/30^2]); 
R = diag([1/5000^2, 1/25^2, 1/25^2]); 

[K, P] = lqr(A_long, B_ctrl, Q, R);
A_cl = A_long - B_ctrl * K; 

%% 2. Traiettorie (Dinamiche Lente)
dxdt = @(t,x) A_cl * x;

% Condizioni iniziali [theta_alta, q=0, u_alta, w=0]
x0 = [deg2rad(15), 0,  40, 0;   
     -deg2rad(20), 0, -30, 0;   
      deg2rad(25), 0, -20, 0]';  

%% 3. Pannello Unico: Ritratto di Fase 2D e Superficie di Lyapunov 3D
figure('Name', 'Analisi del Fugoide (theta vs u)', 'Color', 'w', 'Position', [100 100 1200 600]);

% =========================================================================
% SOTTO-PANNELLO 1: RITRATTO DI FASE 2D
% =========================================================================
subplot(1,2,1);
hold on; grid on;

colori = {'r', 'g', 'b'}; 
for i = 1:size(x0, 2)
    [t, x] = ode45(dxdt, [0 50], x0(:, i)); 
    plot(rad2deg(x(:,1)), x(:,3), 'Color', colori{i}, 'LineWidth', 1.5);
    plot(rad2deg(x(1,1)), x(1,3), '*', 'Color', colori{i}, 'MarkerSize', 8); 
end

[THETA_deg, U_grid] = meshgrid(-100:3:100, -50:2:50);
THETA_rad = deg2rad(THETA_deg);

THETA_dot = zeros(size(THETA_rad));
U_dot = zeros(size(U_grid));
V_contour = zeros(size(THETA_rad));

for i = 1:numel(THETA_rad)
    stato = [THETA_rad(i); 0; U_grid(i); 0];
    derivata = A_cl * stato;
    THETA_dot(i) = derivata(1); 
    U_dot(i) = derivata(3); 
    V_contour(i) = stato' * P * stato;
end

L = sqrt(THETA_dot.^2 + U_dot.^2) + 1e-6;
quiver(THETA_deg, U_grid, rad2deg(THETA_dot)./L, U_dot./L, 0.5, 'Color', [0.7 0.7 0.7]);

livelli = logspace(log10(min(V_contour(:))+1e-3), log10(max(V_contour(:))), 15);
contour(THETA_deg, U_grid, V_contour, livelli, 'LineWidth', 1.5);

xlabel('Pitch Angle $\theta$ [deg]') 
ylabel('Forward Velocity $u$ [ft/s]') 
title('Ritratto di Fase 2D (Dinamica Lenta)')
xlim([-30, 30]); ylim([-50, 50]);

% =========================================================================
% SOTTO-PANNELLO 2: SUPERFICIE DI LYAPUNOV 3D
% =========================================================================
subplot(1,2,2);
hold on; grid on;

% Superficie 3D
V_surf = zeros(size(THETA_rad));
for i = 1:size(THETA_rad, 1)
    for j = 1:size(THETA_rad, 2)
        stato_surf = [THETA_rad(i,j); 0; U_grid(i,j); 0];
        V_surf(i,j) = stato_surf' * P * stato_surf;
    end
end

surf(THETA_deg, U_grid, V_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
colormap jet;
colorbar;

% Traiettorie 3D
for i = 1:size(x0, 2)
    [t, x] = ode45(dxdt, [0 50], x0(:, i));
    V_traiettoria = sum((x * P) .* x, 2); 
    plot3(rad2deg(x(:,1)), x(:,3), V_traiettoria, 'k-', 'LineWidth', 2);
    plot3(rad2deg(x(1,1)), x(1,3), V_traiettoria(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
end

title('Funzione di Lyapunov $V(x) = x^T P x$ (LQR)', 'Interpreter', 'latex')
xlabel('$\theta$ [deg]', 'Interpreter', 'latex')
ylabel('$u$ [ft/s]', 'Interpreter', 'latex')
zlabel('$V(x)$', 'Interpreter', 'latex')
view(-45, 30);

