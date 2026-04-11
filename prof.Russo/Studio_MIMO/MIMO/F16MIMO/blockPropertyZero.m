 % =========================================================================
% VALUTAZIONE ZERI E PROPRIETÀ BLOCCANTE (Sottosistema 2x2)
% =========================================================================
disp('--- Analisi Proprietà Bloccante su Sottosistema 2x2 ---');

% 1. CREAZIONE DEL SOTTOSISTEMA 2x2
n_stati = size(A_long, 1);

% Usiamo solo 2 ingressi: Thrust (1) ed Elevator (2)
ingressi_2x2 = [1, 2];
B_2x2 = B_long(:, ingressi_2x2);

% Usiamo solo 2 uscite: Vt (1) e alpha (2)
uscite_2x2 = [1, 2];
C_2x2 = C_long(uscite_2x2, :);
D_2x2 = D_long(uscite_2x2, ingressi_2x2);

% Modello in Spazio di Stato 2x2
sys_2x2 = ss(A_long, B_2x2, C_2x2, D_2x2);

% 2. CALCOLO DEGLI ZERI INVARIANTI
Zeri_2x2 = tzero(sys_2x2);
disp('Zeri Invarianti del sistema 2x2 (lambda):');
disp(Zeri_2x2);

if isempty(Zeri_2x2)
    error('Ancora nessun zero. Prova a usare un canale SISO (es. solo Elevator su alpha).');
end

% Selezioniamo il primo zero trovato (può essere reale o complesso)
lambda = Zeri_2x2(1); 
fprintf('\nValutazione per lo zero lambda = %f + %fi\n', real(lambda), imag(lambda));

% 3. MATRICE DI ROSENBROCK E NULLSPACE
P_lambda = [lambda * eye(n_stati) - A_long, -B_2x2; 
            C_2x2,                           D_2x2];
        
N = null(P_lambda); 

% Estraiamo le direzioni (N ha n_stati righe per x0, e 2 righe per u0)
x0 = N(1:n_stati, 1);
u0 = N(n_stati+1:end, 1);

disp('Vettore direzione iniziale dello stato (x0):'); disp(x0);
disp('Vettore direzione ingresso bloccante (u0):'); disp(u0);

% =========================================================================
% 4. SIMULAZIONE DINAMICA DELLA PROPRIETÀ BLOCCANTE
% =========================================================================
disp('Avvio simulazione dinamica...');
t = 0:0.01:10; 

% Generazione dell'ingresso bloccante: u(t) = u0 * e^(lambda * t)
% Nota: Se lambda è complesso, la simulazione richiederebbe ingressi complessi 
% (impossibili fisicamente). MATLAB prenderà la parte reale per la simulazione.
u_t = real(u0 * exp(lambda * t))'; 
x0_sim = real(x0);

[y_sim, t_sim, x_sim] = lsim(sys_2x2, u_t, t, x0_sim);

% 5. GRAFICI
figure('Name', 'Proprietà Bloccante 2x2 - F-16', 'Color', 'w');

subplot(2,1,1);
plot(t, u_t, 'LineWidth', 1.5);
title(sprintf('Ingressi Bloccanti ($\\lambda$ = %.2f + %.2fi)', real(lambda), imag(lambda)), 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tempo [s]'); ylabel('Comandi');
legend('Thrust', 'Elevator', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(t, y_sim, 'LineWidth', 1.5);
title('Uscite $V_t$ e $\\alpha$ (Devono essere nulle/piatte)', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tempo [s]'); ylabel('Ampiezza Uscite');
ylim([-0.05 0.05]); 
legend('V_t', '\alpha', 'Location', 'best'); grid on;