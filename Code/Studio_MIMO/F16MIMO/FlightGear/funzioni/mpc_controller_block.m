function u_total = mpc_controller(theta, q, U, W)
%#codegen
% Questo è il controller MPC in Anello Chiuso (Closed-Loop) per Simulink!

% Usa quadprog di MATLAB (non compilarlo in C)
coder.extrinsic('quadprog');
coder.extrinsic('evalin');

% Inizializza l'uscita
u_total = zeros(3, 1);

% --- LETTURA PARAMETRI DAL WORKSPACE ---
% Leggiamo il problema mpc_prob, il trim e i limiti direttamente
% dal workspace di MATLAB dove abbiamo lanciato MPC.m
mpc_prob = evalin('base', 'mpc_prob');
best_theta = evalin('base', 'best_theta');
Vt0 = evalin('base', 'Vt0');
best_u = evalin('base', 'best_u');
dU_max = evalin('base', 'dU_max');

% Variabile persistente per tenere traccia dell'ultimo comando applicato
% (serve per calcolare i limiti di velocità dU/dt)
persistent u_previous;
if isempty(u_previous)
    u_previous = zeros(3,1); % 3 è nu (Thrust, ele, dlef)
end

% --- 1. CALCOLO DELL'ERRORE (Stato per l'MPC) ---
% L'MPC lavora sulle perturbazioni rispetto al trim.
theta_trim = best_theta;
q_trim = 0;
U_trim = Vt0 * cos(best_theta);
W_trim = Vt0 * sin(best_theta);

x_current = [theta - theta_trim;
             q - q_trim;
             U - U_trim;
             W - W_trim];

% --- 2. COSTRUZIONE DELLE MATRICI PER QUADPROG ---
nx = mpc_prob.nx;
nu = mpc_prob.nu;
N  = mpc_prob.N;

% Limiti di derivata (slew rate) 
A_rate = zeros(nu*2*N, mpc_prob.n_vars); 
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

% Condizione iniziale b_eq
beq = zeros(N*nx, 1);
beq(1:nx) = evalin('base', 'A_long_ds') * x_current; 

b_rate = b_rate_base;
b_rate(1:2*nu) = [dU_max + u_previous; dU_max - u_previous];
b_ineq_tot = [mpc_prob.b_ineq_stat; b_rate];

% --- 3. RISOLUZIONE DEL PROBLEMA OTTIMO ---
options = optimoptions('quadprog', 'Display', 'off');
[z_opt, ~, exitflag] = quadprog(mpc_prob.H, mpc_prob.f, A_ineq_tot, b_ineq_tot, mpc_prob.Aeq_base, beq, mpc_prob.lb, mpc_prob.ub, [], options);

if exitflag >= 0
    u_err = z_opt(1:nu);
else
    % Se non trova soluzione (infeasible), mantieni comando precedente
    u_err = u_previous;
end

% Aggiorna la memoria
u_previous = u_err;

% --- 4. CALCOLO COMANDO TOTALE DA INVIARE ALL'AEREO ---
u_total = best_u + u_err;
end
