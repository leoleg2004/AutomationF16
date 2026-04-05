close all
clc
warning off all

%% Importo le librerie di Casadi
addpath('/Users/leonardoleggeri/Desktop/AutomationF16/prof.Russo/Studio_MIMO/MIMO/F16MIMO/casadi')
import casadi.*

%% 1. ESTRAZIONE VARIABILI CON FUNZIONE LQR CLASSICA
nx = size(A_long, 1);
% ATTENZIONE: Prendiamo solo i veri attuatori (Thrust, Elevator, LEF)
B_ctrl = B_long(:, 1:3); % Prende le righe e le prime 3 colonne;%ho filtrato i prime tre elemnti della matrice perche le ultime due ono le raffiche 
%vento e non sono controllabili

nu = size(B_ctrl, 2);

% Pesi delle matrici quadratiche dell' LQR
Q = 100 * eye(nx);
R = 1 * eye(nu);

[K, P, E] = lqr(A_long, B_ctrl, Q, R);

disp('--- Variabili LQR calcolate ---');
disp('Matrice di Riccati P calcolata e pronta per il costo terminale.');

%% 2. MODELLO CASADI (F-16 Longitudinal Dynamics)
x = MX.sym('x', nx);
u = MX.sym('u', nu);
ode = A_long * x + B_ctrl* u; % USO B_ctrl!

Ts = 0.05; % tempo di campionamento
intg_options = struct;
intg_options.tf = Ts;
intg_options.number_of_finite_elements = 5;
dae = struct;
dae.x = x;
dae.p = u;
dae.ode = ode;
intg = integrator('intg', 'rk', dae, intg_options);
res = intg('x0', x, 'p', u); 
x_next = res.xf;
F = Function('F', {x, u}, {x_next}, {'x', 'p'}, {'x_next'}); 

%% 3. PROBLEMA DI CONTROLLO CASADI
opti = casadi.Opti();
N = 100;

% Optimization variables
xf = opti.variable(nx, N+1);
uf = opti.variable(nu, N);
xk = opti.parameter(nx, 1); 

%% Cost function
V = 0;
for j = 1:N
    L = (xf(:,j))' * Q * (xf(:,j)) + (uf(:,j))' * R * (uf(:,j));
    V = V + L;
end
% Costo terminale con Matrice di Riccati P (Miglioramento rispetto all'esempio)
costo_terminale = (xf(:,N+1))' * P * (xf(:,N+1));
V = V + costo_terminale;
opti.minimize(V);

%% Constraints 
opti.subject_to(xf(:,1) == xk);  % Initial condition
% UN UNICO CICLO FOR PER TUTTI I VINCOLI!
for j = 1:N
    % 1. Vincolo Dinamico (predizione dinamica)
    opti.subject_to(xf(:,j+1) == F(xf(:,j), uf(:,j))); 
    
    % 2. Vincolo su q applicato allo stato FUTURO
    opti.subject_to(-0.4 <= xf(2,j+1) <= 0.4);
    
    % 3. Vincolo su w applicato allo stato FUTURO
    opti.subject_to(-10 <= xf(4,j+1) <= 10); 
    
    % [OPZIONALE] Aggiungi qui le saturazioni degli attuatori uf(:,j) se vuoi un vero MPC
end


%% Solver
p_opts = struct('expand', true);
s_opts = struct('max_iter', 1000, 'print_level', 0, 'sb', 'yes');
opti.solver('ipopt', p_opts, s_opts);
OPT_C = opti.to_function('OPT_C', {xk}, {uf, xf, V}, {'xk'}, {'uf_opt', 'xf_opt', 'V_opt'});

%% 4. SIMULAZIONE E PLOT
x0 = [1.5; 1; 10; 5]; 

[uf_sol, xf_sol, V_sol] = OPT_C(x0);
uf = full(uf_sol); 
xf = full(xf_sol);

%% Grafici
figure('Name', 'CasADi: Pura logica MPC ')
subplot(2, 1, 1)
plot(xf(1,:), 'LineWidth', 2); hold on;
plot(xf(2,:), 'LineWidth', 2);
plot(xf(3,:), 'LineWidth', 2);
plot(xf(4,:), 'LineWidth', 2);
legend('\theta (Beccheggio)', 'q (Vel. Beccheggio)', 'U (Vel. X)', 'W (Vel. Z)')
xlabel('samples') 
ylabel('Stati')
grid on;

subplot(2, 1, 2)
for k = 1:nu
    stairs(uf(k,:), 'LineWidth', 2); hold on;
end
legend('Thrust', 'Elevator', 'LEF')
xlabel('samples') 
ylabel('Comandi (u)')
grid on;