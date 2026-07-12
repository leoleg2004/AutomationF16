function [storia_x, storia_u, storia_costo] = simula_mpc_stocastico(mpc_prob, x_iniziale, t_sim, A_long_ds, B_ctrl_ds, dU_max, G, Ts)
    % SIMULA_MPC_STOCASTICO Risolve il problema quadratico e simula l'evoluzione 
    % del sistema perturbato da rumore stocastico (Vento) in anello chiuso.
    nx = mpc_prob.nx;
    nu = mpc_prob.nu;
    N = mpc_prob.N;
    n_vars = mpc_prob.n_vars;
    
    storia_x = zeros(nx, t_sim+1); storia_x(:,1) = x_iniziale;
    storia_u = zeros(nu, t_sim);
    storia_costo = zeros(1, t_sim);
    u_previous = [0;0;0]; 
    options = optimoptions('quadprog', 'Display', 'off');
    % --- Precomputazione Matrici Invarianti ---
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
    % ------------------------------------------

    n_noise = size(G, 2);

    for t = 1:t_sim
        beq = zeros(N*nx, 1);
        beq(1:nx) = A_long_ds * storia_x(:,t); 
        
        b_rate = b_rate_base;
        b_rate(1:2*nu) = [dU_max + u_previous; dU_max - u_previous];
        b_ineq_tot = [mpc_prob.b_ineq_stat; b_rate];
        [z_opt, fval, exitflag] = quadprog(mpc_prob.H, mpc_prob.f, A_ineq_tot, b_ineq_tot, mpc_prob.Aeq_base, beq, mpc_prob.lb, mpc_prob.ub, [], options);
        storia_costo(t) = fval;
        
        if exitflag < 0
            % Invece di interrompere con un fatal error, mostriamo un warning.
            % Il vento potrebbe infatti spingere temporaneamente l'aereo
            % fuori dalla regione feasibile. L'attuatore manterrà
            % l'ultimo comando.
            warning('Infeasible! Il vento ha spinto l''aereo fuori dal CIS allo step %d. Uso controllo precedente.', t);
            if t == 1
                u_applicata = zeros(nu,1);
            else
                u_applicata = u_previous;
            end
        else
            u_applicata = z_opt(1:nu);
        end
        
        storia_u(:, t) = u_applicata;
        
        % --- INIEZIONE MOTO BROWNIANO (TURBOLENZA) ---
        dW_wiener = sqrt(Ts) * randn(n_noise, 1);
        dx_stoch = G * dW_wiener;
        
        % PLANT ANELLO CHIUSO: Aggiornamento stato perturbato
        storia_x(:, t+1) = A_long_ds * storia_x(:,t) + B_ctrl_ds * u_applicata + dx_stoch;
        
        u_previous = u_applicata;
    end
    
    % Nessun plot integrato qui, lasceremo il plotting al main script per pulizia
end
