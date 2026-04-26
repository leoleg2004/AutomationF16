% =========================================================================
% DETERMINISTIC MIMO MPC: RECOVERY VOLO DI CROCIERA F-16 (Con Integratore)
% Dinamica: 4 stati [theta, q, U, W], 3 ingressi [Thrust, Elev, Flap]
% =========================================================================
disp('--- Inizializzazione MPC (Condizione Tranquilla con Tracking Perfetto) ---');

% 1. Definizione dell'Impianto Discreto
Ts = 0.05; 
nx = size(A_long, 1); % 4 stati
nu = size(B_ctrl, 2); % 3 ingressi

Cd = eye(nx); 
Dd = zeros(nx, nu);
sys_c = ss(A_long, B_ctrl, Cd, Dd);

sys_c.OutputName = {'theta', 'q', 'U', 'W'};
sys_c.InputName  = {'Thrust', 'Elevator', 'Flap'};
sys_c.InputGroup.MV = 1:nu;   
sys_c.OutputGroup.MO = 1:nx;  

sys_d = c2d(sys_c, Ts, 'zoh');

% =========================================================================
% 2. CREAZIONE OGGETTO MPC
% =========================================================================
Ky = 80; 
Ku = 20;  
mpcobj = mpc(sys_d, Ts, Ky, Ku);
% =========================================================================
% 3. VINCOLI FISICI
% =========================================================================
deg2rad = pi/180;
mpcobj.MV(1).Min = -5000;
mpcobj.MV(1).Max = 5000;
mpcobj.MV(1).RateMin = -2000; 
mpcobj.MV(1).RateMax = 2000;
mpcobj.MV(2).Min = -25*deg2rad;
mpcobj.MV(2).Max = 25*deg2rad;
mpcobj.MV(2).RateMin = -60*deg2rad*Ts;
mpcobj.MV(2).RateMax = 60*deg2rad*Ts; 
mpcobj.MV(3).Min = -5*deg2rad;
mpcobj.MV(3).Max = 25*deg2rad;
mpcobj.MV(3).RateMin = -40*deg2rad*Ts;
mpcobj.MV(3).RateMax = 40*deg2rad*Ts; 
mpcobj.OV(4).Min = -100; 
mpcobj.OV(4).Max = 100; 

% =========================================================================
% 4. TUNING BILANCIATO
% =========================================================================
% Pesi sulle Uscite: [theta, q, U, W]
% Theta deve andare a zero. U e W sono importanti ma secondari.
mpcobj.Weights.OV = [200, 10, 10, 10]; 

% Pesi sugli Attuatori
mpcobj.Weights.MV = [0, 0, 0]; 

% Pesi sul Rateo degli Attuatori 
% Rimettiamo un po' di "freno" per permettere all'integratore di 
% trovare la posizione di equilibrio millimetrica senza overshoot
mpcobj.Weights.ManipulatedVariablesRate = [0.1, 0.5, 0.5]; 

% =========================================================================
% 5. SETUP SIMULAZIONE (Condizione Iniziale)
% =========================================================================
T_sim = 20; % Aumentiamo a 8 secondi per vedere l'effetto dell'integratore
N = round(T_sim / Ts);
t = (0:N) * Ts;

%[theta; q; U; W]
x0 = [0.2; 0.2; 20; -40];

r_sim = zeros(N, nx); % Riferimento: tornare a 0
x_true = zeros(nx, N+1);  
u_hist = zeros(nu, N+1);  
x_true(:, 1) = x0; 

% Inizializzazione memoria MPC al punto di partenza
xmpc = mpcstate(mpcobj); 
xmpc.Plant = x0; 

disp('Simulazione in corso...');
for i = 1:N
    u_opt = mpcmove(mpcobj, xmpc, x_true(:, i), r_sim(i, :));
    u_hist(:, i) = u_opt;
    
    % Propagazione modello reale
    x_true(:, i+1) = sys_d.A * x_true(:, i) + sys_d.B * u_opt;
end
disp('Simulazione completata!');

% =========================================================================
% 6. GRAFICI
% =========================================================================
rad2deg = 180 / pi;

% --- FIGURA 1: DINAMICA DEGLI STATI ---
figure('Name', 'MPC F-16: Stati a Ciclo Chiuso (Azione Integrale)', 'Color', 'w', 'Position', [100, 100, 800, 600])

subplot(2, 1, 1)
plot(t, x_true(1,:) * rad2deg, 'b', 'LineWidth', 2); hold on; % theta
plot(t, x_true(2,:) * rad2deg, 'r', 'LineWidth', 2);          % q
yline(0, 'k--', 'LineWidth', 1);
legend('\theta (Pitch) [deg]', 'q (Pitch Rate) [deg/s]', 'Location', 'best')
ylabel('Ampiezza [Gradi]')
title('Dinamica Assetto (Inseguimento Perfetto Volo Livellato)')
grid on;

subplot(2, 1, 2)
plot(t, x_true(3,:), 'k', 'LineWidth', 2); hold on; % U
plot(t, x_true(4,:), 'm', 'LineWidth', 2);          % W
yline(0, 'k--', 'LineWidth', 1);
legend('U (Vel. X) [ft/s]', 'W (Vel. Z / \approx \alpha) [ft/s]', 'Location', 'best')
xlabel('Tempo [s]') 
ylabel('Ampiezza [ft/s]')
title('Dinamica Velocità e Incidenza')
grid on;

% --- FIGURA 2: SFORZO DEGLI ATTUATORI ---
figure('Name', 'MPC F-16: Sforzo Attuatori', 'Color', 'w', 'Position', [150, 150, 800, 800])

subplot(3, 1, 1)
plot(t, u_hist(1,:), 'b', 'LineWidth', 2); hold on;
yline(5000, 'r--', 'Max Thrust', 'LabelHorizontalAlignment', 'left'); 
yline(-5000, 'r--', 'Min Thrust', 'LabelHorizontalAlignment', 'left');
ylabel('Spinta [lbs]')
title('Azione di Controllo: Manetta (Thrust)')
grid on;

subplot(3, 1, 2)
plot(t, u_hist(2,:) * rad2deg, 'r', 'LineWidth', 2); hold on;
yline(25, 'k--', 'Saturazione (+25°)'); 
yline(-25, 'k--', 'Saturazione (-25°)');
ylabel('Deflessione [deg]')
title('Azione di Controllo: Equilibratore')
grid on;

subplot(3, 1, 3)
plot(t, u_hist(3,:) * rad2deg, 'g', 'LineWidth', 2); hold on;
yline(25, 'k--', 'Saturazione (+25°)'); 
yline(-5, 'k--', 'Saturazione (-25°)');
xlabel('Tempo [s]') 
ylabel('Deflessione [deg]')
title('Azione di Controllo: Leading Edge Flap')
grid on;