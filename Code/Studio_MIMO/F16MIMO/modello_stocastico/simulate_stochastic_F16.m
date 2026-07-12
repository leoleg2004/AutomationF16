function [t, X, V2_ito, tassa_accumulata] = simulate_stochastic_F16(A, Bctrl, G, U_matrix, x0, t_final, dt)
    % SIMULATE_STOCHASTIC_F16 Simula il modello stocastico longitudinale dell'F-16
    % Utilizza il metodo numerico di Eulero-Maruyama per risolvere l'Equazione
    % Differenziale Stocastica (SDE): dx = (Ax + Bctrl*u)dt + G dW
    %
    % Output:
    % t: Vettore dei tempi
    % X: Matrice degli stati simulati (4 righe x N step temporali)
    % V2_ito: Integrazione stocastica di V^2 tramite Lemma di Itô
    % tassa_accumulata: Valore cumulativo nel tempo della tassa di Itô

    % 1. Inizializzazione dei vettori
    t = 0:dt:t_final;
    N = length(t);
    n_states = length(x0);
    n_noise = size(G, 2); 
    
    X = zeros(n_states, N);
    X(:, 1) = x0;
    
    % Variabile per integrare V^2 con il Lemma di Itô
    V2_ito = zeros(1, N);
    V2_ito(1) = x0(3)^2 + x0(4)^2;
    
    % Variabile per l'avanzamento della tassa di Itô
    tassa_accumulata = zeros(1, N);
    tassa_accumulata(1) = 0;
    
    % Calcolo della "Tassa di Itô"
    % La varianza del rumore su U e W corrisponde alla somma dei quadrati 
    % degli elementi delle rispettive righe della matrice di diffusione G.
    sigma2_U = sum(G(3,:).^2);
    sigma2_W = sum(G(4,:).^2);
    tassa_ito = sigma2_U + sigma2_W;
    
    if size(U_matrix, 2) ~= N
        error('U_matrix deve avere lo stesso numero di colonne dei passi temporali N');
    end
    
    % 2. Integrazione con metodo di Eulero-Maruyama
    for k = 1:(N-1)
        % Processo di Wiener
        dW_wiener = sqrt(dt) * randn(n_noise, 1);
        
        u_k = U_matrix(:, k);
        
        % Differenziali deterministico e stocastico
        dx_det = (A * X(:, k) + Bctrl * u_k) * dt;
        dx_stoch = G * dW_wiener;
        
        % Incremento totale per U e W (per la formula di Itô)
        dU = dx_det(3) + dx_stoch(3);
        dW = dx_det(4) + dx_stoch(4);
        
        % Aggiornamento stato
        X(:, k+1) = X(:, k) + dx_det + dx_stoch;
        
        % Integrazione numerica di Itô per V^2
        % d(V^2) = 2*U*dU + 2*W*dW + (sigma_U^2 + sigma_W^2)dt
        U_current = X(3, k);
        W_current = X(4, k);
        V2_ito(k+1) = V2_ito(k) + 2 * U_current * dU + 2 * W_current * dW + tassa_ito * dt;
        
        % Avanzamento della tassa (Drift energetico)
        tassa_accumulata(k+1) = tassa_accumulata(k) + tassa_ito * dt;
    end
end
