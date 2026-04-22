% =========================================================================
% DETERMINISTIC MIMO MPC PER F-16 LONGITUDINALE
% =========================================================================
disp('--- Inizializzazione Deterministic MPC ---');

% 1. Definizione dell'Impianto Discreto
Ts = 0.05; 
nx = size(A_long, 1); % 4 stati
nu = size(B_ctrl, 2); % 3 ingressi

% Assumiamo di misurare in modo perfetto tutti e 4 gli stati (C = Identità)
Cd = eye(nx); 
Dd = zeros(nx, nu);

% Creazione del sistema State-Space continuo
sys_c = ss(A_long, B_ctrl, Cd, Dd);

% --- METODO INFALLIBILE PER DEFINIRE LE VARIABILI ---
% Usiamo le proprietà native dell'oggetto ss di MATLAB
sys_c.InputGroup.MV = 1:nu;   % Diciamo che tutti i 3 ingressi sono Manipulated Variables
sys_c.OutputGroup.MO = 1:nx;  % Diciamo che tutte le 4 uscite sono Measured Outputs

% Assegniamo i nomi fisici (Aiuta l'MPC a generare grafici più chiari!)
sys_c.InputName  = {'Thrust', 'Elevator', 'Flap'};
sys_c.OutputName = {'Vt', 'alpha', 'q', 'theta'};

% Discretizzazione
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A; 
Bd = sys_d.B;

% =========================================================================
% 2. CREAZIONE OGGETTO MPC
% =========================================================================
Ky = 20; % Orizzonte di Predizione (Prediction Horizon)
Ku = 5;  % Orizzonte di Controllo (Control Horizon)

% Creazione del controllore
% Uso sys_d.Ts per garantire che il tempo di campionamento combaci matematicamente
mpcobj = mpc(sys_d, sys_d.Ts, Ky, Ku);

disp('Oggetto MPC creato con successo!');
% =========================================================================
% 3. PROTEZIONE DELL'INVILUPPO E VINCOLI FISICI (Envelope Protection)
% =========================================================================
deg2rad = pi/180;

% --- Vincoli sugli Attuatori (Manipulated Variables - MV) ---
% 1. Manetta (Thrust)
mpcobj.MV(1).Min = -5000; 
mpcobj.MV(1).Max =  5000;
mpcobj.MV(1).RateMin = -2000; % Max delta-spinta per step
mpcobj.MV(1).RateMax =  2000;

% 2. Equilibratore (Elevator)
mpcobj.MV(2).Min = -25 * deg2rad; 
mpcobj.MV(2).Max =  25 * deg2rad;
mpcobj.MV(2).RateMin = -60 * deg2rad * Ts; % Limite idraulico (60 deg/s)
mpcobj.MV(2).RateMax =  60 * deg2rad * Ts; 

% 3. Leading Edge Flap (LEF)
mpcobj.MV(3).Min = -5  * deg2rad; 
mpcobj.MV(3).Max =  25 * deg2rad;
mpcobj.MV(3).RateMin = -40 * deg2rad * Ts; 
mpcobj.MV(3).RateMax =  40 * deg2rad * Ts; 

% --- Vincoli sugli Stati/Uscite (Output Variables - OV) ---
% Protezione vitale contro lo STALLO (Limite su alpha, stato n° 2)
mpcobj.OV(2).Min = -10 * deg2rad; 
mpcobj.OV(2).Max =  20 * deg2rad; % L'MPC non farà MAI superare i 20 gradi

% =========================================================================
% 4. TUNING DEI PESI (Matrici Q ed R dell'ottimizzatore)
% =========================================================================
% Priorità di Tracking: Vogliamo seguire il beccheggio (q, stato 3) e tenere alpha a bada (stato 2)
mpcobj.Weights.OV = [1, 10, 5, 1]; % [Vt, alpha, q, theta]

% Costo Energia: Quanto "costa" usare l'attuatore (Priorità all'uso dell'equilibratore)
mpcobj.Weights.MV = [0.1, 0.1, 0.1]; 

% Costo Usura: Penalizza variazioni brusche dei comandi
mpcobj.Weights.ManipulatedVariablesRate = [0.1, 0.5, 0.5]; 

% =========================================================================
% 5. SIMULAZIONE (Main Loop)
% =========================================================================
T_sim = 10; % Secondi
N = round(T_sim / Ts);
t_hist = (0:N) * Ts;

% Vettore Riferimenti (Vogliamo una cabrata di 10 deg/s)
r_sim = zeros(N, nx);
r_sim(round(1/Ts):end, 3) = 10 * deg2rad; % Step sul beccheggio a t=1s

% Memorie
x_true = zeros(nx, N+1);  
u_hist = zeros(nu, N+1);  

% Inizializza lo stato interno dell'MPC
xmpc = mpcstate(mpcobj); 

disp('Simulazione Deterministica in corso...');
figure('Name', 'Deterministic MIMO MPC F-16', 'Color', 'w', 'Position', [100 100 800 600]);

for i = 1:N
    % 1. L'MPC calcola la mossa ottima basandosi ESATTAMENTE sullo stato reale
    u_opt = mpcmove(mpcobj, xmpc, x_true(:, i), r_sim(i, :)');
    u_hist(:, i) = u_opt;
    
    % 2. Propagazione della Fisica (Senza alcun rumore o vento)
    x_true(:, i+1) = Ad * x_true(:, i) + Bd * u_opt;
    
    % 3. Plot Live
    if mod(i, 5) == 0 || i == N
        clf;
        % Plot Stati (Beccheggio e Alpha)
        subplot(2,1,1); hold on; grid on;
        plot(t_hist(1:N), r_sim(:, 3), 'r--', 'LineWidth', 2); % Rif. q
        plot(t_hist(1:i+1), x_true(3, 1:i+1), 'k', 'LineWidth', 1.5); % q
        plot(t_hist(1:i+1), x_true(2, 1:i+1), 'b', 'LineWidth', 1.5); % alpha
        yline(20*deg2rad, 'b--', 'Limite Alpha (Stallo)');
        title('Dinamica F-16: Beccheggio (q) e Incidenza (\alpha)');
        legend('Rif. q', 'q (rad/s)', '\alpha (rad)', 'Location', 'best');
        
        % Plot Comandi (Equilibratore)
        subplot(2,1,2); hold on; grid on;
        plot(t_hist(1:i+1), u_hist(2, 1:i+1), 'b', 'LineWidth', 1.5);
        yline(25*deg2rad, 'r--', 'Saturazione Eq.'); 
        yline(-25*deg2rad, 'r--');
        title('Sforzo di Controllo (Equilibratore)');
        xlabel('Tempo (s)'); ylabel('\delta_e (rad)');
        drawnow;
    end
end
disp('Simulazione completata!');