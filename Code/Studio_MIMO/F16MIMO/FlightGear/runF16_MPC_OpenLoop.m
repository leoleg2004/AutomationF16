% =========================================================================
% Tesi Triennale - Simulazione F-16 Simulink con comandi MPC
% Ing. Leggeri Leonardo
% =========================================================================

% Svuota cache Simulink per evitare errori di Shadowed File dopo lo spostamento
bdclose('all');
clear F16_MPC;
rehash;

% 1. Prepara il file Simulink caricando i parametri di linearizzazione
disp('Avvio Simulazione Matematica dell''MPC...');
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
startup_project();

disp('Caricamento costanti di conversione...');
conversion; 

% 1. Verifica che i dati MPC siano presenti nel workspace
if ~exist('storia_u', 'var') || ~exist('Ts', 'var')
    disp('Dati MPC non trovati. Assicurati di aver eseguito MPC.m prima!');
    return;
end

% 2. Assicurati che i dati di trim siano presenti
if ~exist('best_u', 'var') || ~exist('best_theta', 'var') || ~exist('Vt0', 'var')
    disp('Dati di Trim non trovati. Calcolo in corso...');
    Param = load_F16_params();
    TrimF16; 
end

% 3. Impostazione delle Condizioni Iniziali (IC) per Simulink
h0 = -4000 * ft2m; % Altitudine
IC.inertial_position = [0, 0, h0]; 

if exist('x_iniziale', 'var')
    disp('Applicazione condizione iniziale (x_iniziale) dall''MPC...');
    theta_init = best_theta + x_iniziale(1);
    q_init = x_iniziale(2);
    U_trim = Vt0 * cos(best_theta);
    W_trim = Vt0 * sin(best_theta);
    IC.body_velocity = [U_trim + x_iniziale(3), 0, W_trim + x_iniziale(4)]; 
    IC.euler_angles = [0, theta_init, 0];  
    IC.omega = [0, q_init, 0]; 
else
    IC.body_velocity = [Vt0 * cos(best_theta), 0, Vt0 * sin(best_theta)]; 
    IC.euler_angles = [0, best_theta, 0];  
    IC.omega = [0, 0, 0];            
end

% 4. Preparazione delle traiettorie Timeseries per Simulink
% ESTENSIONE DEL TEMPO DI SIMULAZIONE (FlightGear)
% L'MPC calcola magari solo 10 secondi, estendiamo i comandi mantenendo
% l'ultimo valore costante per dare tempo al modello di stabilizzarsi visivamente.
tempo_desiderato_fg = 40; % 40 secondi totali per FlightGear
step_desiderati = round(tempo_desiderato_fg / Ts);

if size(storia_u, 2) < step_desiderati
    pad_len = step_desiderati - size(storia_u, 2);
    ultimo_u = storia_u(:, end);
    storia_u_fg = [storia_u, repmat(ultimo_u, 1, pad_len)];
else
    storia_u_fg = storia_u;
end

num_steps = size(storia_u_fg, 2);
sim_time = (0 : num_steps-1)' * Ts;

% L'MPC calcola perturbazioni, sommiamo i valori di trim
Thrust_val = best_u(1) + storia_u_fg(1, :)';
ele_val    = best_u(2) + storia_u_fg(2, :)'; % In radianti (il blocco dice 'rad')
dlef_val   = best_u(3) + storia_u_fg(3, :)'; % In radianti (il blocco dice 'rad')

% Creazione oggetti timeseries leggibili direttamente da "From Workspace"
Thrust_ts = timeseries(Thrust_val, sim_time);
ele_ts    = timeseries(ele_val, sim_time);
dlef_ts   = timeseries(dlef_val, sim_time);
ail_ts    = timeseries(zeros(num_steps, 1), sim_time);
rud_ts    = timeseries(zeros(num_steps, 1), sim_time);
gust_ts   = timeseries(zeros(num_steps, 3), sim_time); % La raffica (gust) è un vettore 3D!

disp('Traiettoria dei comandi MPC generata nel workspace (timeseries).');

% 5. Esecuzione automatica
Tend = max(sim_time);
disp(['Simulazione Open-Loop in Simulink per Tend = ', num2str(Tend), 's...']);
simout = sim('F16_MPC', 'StopTime', num2str(Tend)); 
plot_trajectories(simout);

disp('Simulazione completata. Avvio riproduzione su FlightGear...');
play_flightgear(simout);
