% =========================================================================
% VALUTAZIONE ZERI INVARIANTI E PROPRIETÀ BLOCCANTE (SISTEMA LONGITUDINALE)
% =========================================================================
disp('--- Analisi Zeri Invarianti e Proprietà Bloccante ---');

% Creazione dell'oggetto State-Space con le matrici complete dell'impianto
sys_long = ss(A_long, B_long, C_long, D_long);
n_stati = size(A_long, 1);

% 1. Calcolo degli Zeri Invarianti
Zeri_MIMO = tzero(sys_long);
disp('Zeri Invarianti del sistema (lambda):');
disp(Zeri_MIMO);

if isempty(Zeri_MIMO)
    error('Il sistema non ha zeri invarianti. Proprietà bloccante non applicabile.');
end

% 2. Selezioniamo il primo zero
lambda = Zeri_MIMO(1); 
fprintf('\nValutazione per lo zero lambda = %f + %fi\n', real(lambda), imag(lambda));

% Costruzione della Matrice di Rosenbrock P(lambda)
P_lambda = [lambda * eye(n_stati) - A_long, -B_long; 
            C_long,                          D_long];
        
% 3. Calcolo dello Spazio Nullo (Nullspace)
N = null(P_lambda); 
if isempty(N)
    error('Nessun nullspace trovato. Lo zero potrebbe essere degenere.');
end

% Estraiamo le direzioni di stato e ingresso dal nullspace
x0 = N(1:n_stati, 1);
u0 = N(n_stati+1:end, 1);
disp('Vettore direzione iniziale dello stato (x0):'); disp(x0);
disp('Vettore direzione ingresso bloccante (u0):'); disp(u0);

% Verifica Algebrica: ||P(lambda) * [x0; u0]|| deve essere prossimo a 0 (es. 10^-15)
residuo = norm(P_lambda * [x0; u0]);
fprintf('Residuo algebrico ||P_lambda * [x0; u0]||: %e\n\n', residuo);

% =========================================================================
% 4. SIMULAZIONE DINAMICA E VERIFICA PROPRIETÀ BLOCCANTE
% =========================================================================
disp('Avvio simulazione dinamica per verificare che l''uscita sia nulla...');
t = 0:0.01:10; % Vettore tempo di 10 secondi

% Generazione dell'ingresso bloccante u(t) = u0 * e^(lambda * t)
% Nota: trasponiamo (') perché lsim richiede [tempo x numero_ingressi]
u_t = real(u0 * exp(lambda * t))'; 
x0_sim = real(x0);

% Simulazione con 'lsim'
[y_sim, t_sim, x_sim] = lsim(sys_long, u_t, t, x0_sim);

% 5. Tracciamento dei Grafici
figure('Name', 'Proprietà Bloccante MIMO - F-16', 'Color', 'w');

% Plot 1: Ingressi Bloccanti (Thrust, Elevator, LEF, Gust X, Gust Z)
subplot(2,1,1);
plot(t, u_t, 'LineWidth', 1.5);
title(sprintf('Ingresso Bloccante $u(t) = u_0 e^{\\lambda t}$ ($\\lambda$ = %.2f + %.2fi)', real(lambda), imag(lambda)), 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tempo [s]');
ylabel('Ampiezza Ingressi');
legend('Thrust', 'Elevator', 'LEF', 'Gust X', 'Gust Z', 'Location', 'best');
grid on;

% Plot 2: Uscite Misurate (Vt, alpha, q, xbdd, zbdd, nz) -> Devono essere piatte a 0!
subplot(2,1,2);
plot(t, y_sim, 'LineWidth', 1.5);
title('Risposta del Sistema $y(t)$ - \textbf{Uscite Nulle}', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tempo [s]');
ylabel('Ampiezza Uscite');
ylim([-0.05 0.05]); % Zoom massiccio per dimostrare l'annullamento
legend('V_t', '\alpha', 'q', 'x_{bdd}', 'z_{bdd}', 'n_z', 'Location', 'best');
grid on;