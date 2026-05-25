clear;
clc;
close all;

set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'defaultAxesFontSize', 14);
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'defaulttextInterpreter', 'latex');
rng('default')


%% 0. Parametri del sistema
g = 9.81;
l = 0.5;
m = 0.1;
b = 1e-2;

%% 1. Ritratto di fase del pendolo inverso senza attrito
% (sistema autonomo)
b = 0; % no attrito
u = 0; % no azione di controllo/accelerazione


%ODE del sistema
dxdt= @(t,x) pendulum(t,x,u,g,l,m,b); 
% assegno alla variabile dxdt il richiamo della funzione, dove gli ultimi 4 li conosco, i primi due (t,x) li diamo 
% come simboliche che verranno popolate in un successivo momento quando chiamo dxdt

x0 = [0 0;
      0.3 2;
      pi/2 0]'; % set di condizioni iniziali del sistema

figure(1)
grid on;
hold on;
for i=1:3
    [tt, xx] = ode45(dxdt, [0 10], x0(:, i)); % avanzamento del sistema
    
    plot(atan2(sin(xx(:,1)), cos(xx(:,1))), xx(:,2)); %vogliamo vedere l'andamento della posizione nel cerchio tra -pi e +pi e non in funzione del tempo (con andamento sinusoidale o cosinusoidale)
    
    drawnow;
end
title('\textbf{Traiettorie senza attrito}')
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi pi])
%legend('Interpreter','latex');

% Tracciamo il ritratto di fase
[x1, x2]= meshgrid(-pi:0.3:pi, -8:0.5:8);
x1_dot = x2;
x2_dot = -g/l .*sin(x1);
quiver(x1,x2, x1_dot, x2_dot);
legend({'$x_0 = (0, 0)$', ...
        '$x_0 = (0.3, 2)$', ...
        '$x_0 = (\pi/2, 0)$', ...
        'Ritratto di fase'}, 'Interpreter', 'latex')

%% 2. Ritratto di fase del pendolo con attrito

% (sistema autonomo)
b = 1e-2; % attrito
u = 0; % no azione di controllo/accelerazione


%ODE del sistema
dxdt= @(t,x) pendulum(t,x,u,g,l,m,b); 
% assegno alla variabile dxdt il richiamo della funzione, dove gli ultimi 4 li conosco, i primi due (t,x) li diamo 
% come simboliche che verranno popolate in un successivo momento quando chiamo dxdt

x0 = [0 0;
      0.3 2;
      pi/2 0]'; % set di condizioni iniziali del sistema

figure(2)
grid on;
hold on;
for i=1:3
    [tt, xx] = ode45(dxdt, [0 10], x0(:, i)); % avanzamento del sistema
    
    plot(atan2(sin(xx(:,1)), cos(xx(:,1))), xx(:,2)); %vogliamo vedere l'andamento della posizione nel cerchio tra -pi e +pi e non in funzione del tempo (con andamento sinusoidale o cosinusoidale)
    
    drawnow;
end
title('\textbf{Traiettorie con attrito}')
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi pi])
%legend('Interpreter','latex');

% Tracciamo il ritratto di fase
[x1, x2]= meshgrid(-pi:0.3:pi, -8:0.5:8);
x1_dot = x2;
x2_dot = -g/l .*sin(x1) - b/(m*l^2)*x2;
quiver(x1,x2, x1_dot, x2_dot);
legend({'$x_0 = (0, 0)$', ...
        '$x_0 = (0.3, 2)$', ...
        '$x_0 = (\pi/2, 0)$', ...
        'Ritratto di fase'}, 'Interpreter', 'latex')

%% 3. Funzione di Lyapunov per il pendolo inverso 

% Energia del sistema
V=@(x_1,x_2) 1/2 * (m*l^2)*(x_2.^2) + m*g*l*(1-cos(x_1));


% Verifichiamo che l'equazione decresca sempre nell'ultima
% simulazione del pendolo
N = size(xx, 1);
Vx = zeros(N,1);
for i = 1:N
    Vx(i) = V(xx(i, 1), xx(i, 2));
end

figure(3)
plot(Vx)

% Plottare la curva di livello della funzione sul ritratto di fase
figure(2)

[x1, x2]= meshgrid(-pi:0.3:pi, -8:0.5:8);
V_contour = V(x1,x2);
contour(x1,x2,V_contour, 10, 'LineWidth',2);
legend({'$x_0 = (0, 0)$', ...
        '$x_0 = (0.3, 2)$', ...
        '$x_0 = (\pi/2, 0)$', ...
        'Ritratto di fase',...
        'Contour $V(x)$'}, 'Interpreter', 'latex')

%% 3b (APPROFONDIMENTO). Visualizzazione 3D della Funzione di Lyapunov

figure(4)
clf; % Pulisce la figura corrente
hold on;
grid on;

% 1. Generiamo la superficie 3D di V(x)
[X1, X2] = meshgrid(-pi:0.1:pi, -8:0.2:8); % Griglia più fitta per un grafico fluido
V_surf = 1/2 * (m*l^2)*(X2.^2) + m*g*l*(1-cos(X1));

% Disegniamo la superficie (usiamo l'effetto trasparenza 'FaceAlpha' per vedere sotto)
surf(X1, X2, V_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
colormap jet;
colorbar; % Mostra la scala dei valori di energia

% 2. Calcoliamo i valori di V lungo la traiettoria reale dell'ultima simulazione
% (Riprendiamo xx dall'ultimo ciclo for della sezione 2 con attrito)
theta_traiettoria = atan2(sin(xx(:,1)), cos(xx(:,1))); % normalizzato tra -pi e pi
omega_traiettoria = xx(:,2);
V_traiettoria = zeros(size(xx,1), 1);

for i = 1:size(xx,1)
    V_traiettoria(i) = V(theta_traiettoria(i), omega_traiettoria(i));
end

% 3. Plottiamo la traiettoria dello stato TRIDIMENSIONALE sopra la superficie
% Usiamo una linea spessa nera o rossa per farla risaltare
plot3(theta_traiettoria, omega_traiettoria, V_traiettoria, 'k-', 'LineWidth', 3);

% Evidenziamo il punto di partenza (x0) e il punto di arrivo
plot3(theta_traiettoria(1), omega_traiettoria(1), V_traiettoria(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
plot3(theta_traiettoria(end), omega_traiettoria(end), V_traiettoria(end), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

% Grafica e Labels
title('\textbf{Funzione di Lyapunov $V(x)$ in 3D e traiettoria con attrito}', 'Interpreter', 'latex')
xlabel('$\theta$ [rad]', 'Interpreter', 'latex')
ylabel('$\dot{\theta}$ [rad/s]', 'Interpreter', 'latex')
zlabel('$V(x)$ (Energia)', 'Interpreter', 'latex')

view(-35, 30); % Ruota la visuale 3D per una prospettiva ottimale
legend({'Superficie $V(x)$', 'Traiettoria del sistema', 'Inizio ($x_0$)', 'Fine'}, 'Interpreter', 'latex', 'Location', 'best')

%% 4. Invariant set per il sistema controllato
% Sampling time
Ts = 0.1; 

% Define the controlled system dynamics
A = [1 Ts;
    Ts*g/l -Ts*b/(m*l^2)];
B = [0;
    Ts/l];

% Vincoli su stato e ingresso
Hx = [eye(2); 
    -eye(2)];

hx = [pi;
      1;
      pi;
      1];

Hu = [2; -2];

hu = 5*ones(2,1);

% Q, R del costo quadratico (LQR)
Q = eye(2);
R = 1;

% Calcolo 
[G, g] = cis(A,B, [0; 0], 0, Hx, hx, Hu, hu, Q, R);
CIS  = Polyhedron(G,g);
figure(5)
CIS.plot()
title('Control Invariant Set del sistema linearizzato')
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi, pi])
ylim([-2,2])

%% 5. N-step-controllable set (del set invariante)
Np = 10; 

[Np_steps_H, Np_steps_h] = controllable_set(Hx,hx,Hu, hu, G,g, A, B, Np);
Np_step_set = Polyhedron(Np_steps_H, Np_steps_h);

figure(5)
Np_step_set.plot('Alpha', 0);
hold on;
CIS.plot()
title(sprintf('CIS e %d_step_controllable set', Np))
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi, pi])
ylim([-2,2])

legend({sprintf('%d-step-controllable set', Np), 'Control-invariant-set'})

Np = 50; 

[Np_steps_H, Np_steps_h] = controllable_set(Hx,hx,Hu, hu, G,g, A, B, Np);
Np_step_set = Polyhedron(Np_steps_H, Np_steps_h);

figure(6)
Np_step_set.plot('Alpha', 0);
hold on;
CIS.plot()
title(sprintf('CIS e %d_step_controllable set', Np))
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi, pi])
ylim([-2,2])

legend({sprintf('%d-step-controllable set', Np), 'Control-invariant-set'})

Np = 100; 

[Np_steps_H, Np_steps_h] = controllable_set(Hx,hx,Hu, hu, G,g, A, B, Np);
Np_step_set = Polyhedron(Np_steps_H, Np_steps_h);

figure(7)
Np_step_set.plot('Alpha', 0);
hold on;
CIS.plot()
title(sprintf('CIS e %d_step_controllable set', Np))
xlabel('$\theta$ [rad]')
ylabel('$\dot{\theta}$ [rad/s]')
xlim([-pi, pi])
ylim([-2,2])

legend({sprintf('%d-step-controllable set', Np), 'Control-invariant-set'})