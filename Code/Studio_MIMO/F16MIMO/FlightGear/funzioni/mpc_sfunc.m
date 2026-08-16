function mpc_sfunc(block)
% S-Function MATLAB di Livello-2 che implementa l'MPC
% Da usare in Simulink con il blocco "Level-2 MATLAB S-Function"

    setup(block);

function setup(block)
    % Definizione porte
    block.NumInputPorts  = 4; % theta, q, U, W
    block.NumOutputPorts = 3; % Thrust, ele, dlef
    
    % Configurazione porte
    for i = 1:4
        block.InputPort(i).Dimensions = 1;
        block.InputPort(i).DirectFeedthrough = true;
    end
    
    for i = 1:3
        block.OutputPort(i).Dimensions = 1;
    end
    
    % Tempo di campionamento (discreto a Ts = 0.1)
    Ts = evalin('base', 'Ts');
    block.SampleTimes = [Ts 0];
    
    block.SimStateCompliance = 'DefaultSimState';
    
    % Metodi di callback
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);

function DoPostPropSetup(block)
    % Spazio per le memorie di stato (ultimo comando u applicato)
    block.NumDworks = 1;
    block.Dwork(1).Name            = 'u_previous';
    block.Dwork(1).Dimensions      = 3;
    block.Dwork(1).DatatypeID      = 0; % double
    block.Dwork(1).Complexity      = 'Real';

function Start(block)
    % Inizializza l'ultimo comando a zero (perturbazione nulla all'inizio)
    block.Dwork(1).Data = [0; 0; 0];

function Outputs(block)
    % --- 1. LEGGI STATO ATTUALE DA SIMULINK ---
    theta = block.InputPort(1).Data;
    q = block.InputPort(2).Data;
    Vt = block.InputPort(3).Data; % Arriva in m/s e rimane in m/s
    alpha = block.InputPort(4).Data;
    
    U = Vt * cos(alpha);
    W = Vt * sin(alpha);
    
    % --- 2. RECUPERA PARAMETRI (Ottimizzato con persistent per non ricalcolare) ---
    persistent p;
    if isempty(p)
        p.mpc_prob = evalin('base', 'mpc_prob');
        p.best_theta = evalin('base', 'best_theta');
        
        % best_u nel workspace è 1x4: [Thrust, ele, lef, theta].
        % Dobbiamo estrarre SOLO i primi 3 e renderlo un vettore colonna 3x1.
        raw_best_u = evalin('base', 'best_u');
        p.best_u = [raw_best_u(1); raw_best_u(2); raw_best_u(3)];
        
        p.Vt0 = evalin('base', 'Vt0');
        p.A_long_ds = evalin('base', 'A_long_ds');
        p.dU_max = evalin('base', 'dU_max');
        p.options = optimoptions('quadprog', 'Display', 'off');
    end
    
    % Comando precedente dalla memoria
    u_prev = block.Dwork(1).Data;
    
    % --- 3. CALCOLA L'ERRORE (x_err) ---
    theta_trim = p.best_theta;
    q_trim = 0;
    U_trim = p.Vt0 * cos(p.best_theta);
    W_trim = p.Vt0 * sin(p.best_theta);

    x_current = [theta - theta_trim;
                 q - q_trim;
                 U - U_trim;
                 W - W_trim];
                 
    % --- 4. COSTRUISCI MATRICI QUADPROG (Solo la parte dinamica) ---
    nx = p.mpc_prob.nx;
    nu = p.mpc_prob.nu;
    N  = p.mpc_prob.N;

    A_rate = zeros(nu*2*N, p.mpc_prob.n_vars); 
    b_rate_base = zeros(nu*2*N, 1);
    for k = 1:N
        idx_u = (k-1)*nu+1:k*nu;
        A_rate((k-1)*2*nu+1:k*2*nu, idx_u) = [eye(nu); -eye(nu)];
        if k > 1
            idx_u_prev = (k-2)*nu+1:(k-1)*nu;
            A_rate((k-1)*2*nu+1:k*2*nu, idx_u_prev) = [-eye(nu); eye(nu)];
            b_rate_base((k-1)*2*nu+1:k*2*nu) = [p.dU_max; p.dU_max];
        end
    end
    A_ineq_tot = [p.mpc_prob.A_ineq_stat; A_rate];

    beq = zeros(N*nx, 1);
    beq(1:nx) = p.A_long_ds * x_current; 

    b_rate = b_rate_base;
    b_rate(1:2*nu) = [p.dU_max + u_prev; p.dU_max - u_prev];
    b_ineq_tot = [p.mpc_prob.b_ineq_stat; b_rate];

    % --- 5. RISOLVI IL PROBLEMA ---
    [z_opt, ~, exitflag] = quadprog(p.mpc_prob.H, p.mpc_prob.f, A_ineq_tot, b_ineq_tot, p.mpc_prob.Aeq_base, beq, p.mpc_prob.lb, p.mpc_prob.ub, [], p.options);

    if exitflag >= 0
        u_err = z_opt(1:nu);
    else
        u_err = u_prev; % Infeasible, mantieni costante
    end
    
    % Salva in memoria per il prossimo step
    block.Dwork(1).Data = u_err;
    
    % --- 6. INVIA IL COMANDO TOTALE ALL'AEREO ---
    u_tot = p.best_u + u_err;
    
    % Saturazione di Sicurezza Assoluta (Anti-NaN per tabelle aerodinamiche)
    u_tot(1) = max(1000 * 4.44822, min(19000 * 4.44822, u_tot(1))); % Thrust [Newton]
    u_tot(2) = max(-0.4363, min(0.4363, u_tot(2))); % Elevator [-25, 25] gradi in radianti
    u_tot(3) = max(0, min(0.4363, u_tot(3))); % Flaps (LEF) [0, 25] gradi in radianti
    
    if any(isnan(u_tot))
        fprintf('\n--- ERRORE MPC ---\nu_tot contiene NaN!\nu_err: [%f, %f, %f]\nbest_u: [%f, %f, %f]\n', u_err, p.best_u);
        fprintf('Stato attuale x_current: [%f, %f, %f, %f]\n', x_current);
        error('MPC ha generato comandi NaN!');
    end
    
    % Manda le uscite direttamente
    block.OutputPort(1).Data = u_tot(1); % Thrust (N)
    block.OutputPort(2).Data = u_tot(2); % Elevator (rad)
    block.OutputPort(3).Data = u_tot(3); % Flaps (rad)

function Update(block)
    % (L'aggiornamento memoria avviene in Outputs per comodità in questo caso)
