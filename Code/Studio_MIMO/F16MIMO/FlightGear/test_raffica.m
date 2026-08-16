% =========================================================================
% Tesi Triennale - Simulazione F-16 Simulink con comandi MPC
% Ing. Leggeri Leonardo
% =========================================================================

% Svuota cache Simulink
bdclose('all');
clear F16_MPC;
rehash;

% 1. Prepara l'ambiente
disp('Avvio Simulazione Matematica dell''MPC (Closed-Loop)...');
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
startup_project();
conversion; 

% Verifica che i parametri MPC (mpc_prob, ecc.) siano nel workspace
if ~exist('mpc_prob', 'var')
    disp('Dati MPC non trovati. Esegui MPC.m prima!');
    return;
end

% 2. Impostazione delle Condizioni Iniziali (IC) per Simulink
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

% Il tempo di simulazione per Simulink ora è libero!
Tend = 40; % secondi 
disp(['Simulazione Closed-Loop in Simulink per Tend = ', num2str(Tend), 's...']);

% --- TURBOLENZA REALISTICA E DISTURBI ---
% Creiamo un rumore di fondo (turbolenza) per l'intero volo
t_sim_array = (0:0.1:Tend)';
N_samples = length(t_sim_array);

% Turbolenza laterale e verticale (vento)
% Usiamo una combinazione di rumore bianco a bassa ampiezza e sinusoidi a bassa frequenza
noise_v = 1.0 * randn(N_samples, 1) + 2.0 * sin(0.5 * t_sim_array); % Vento asse Y (m/s)
noise_w = 0.5 * randn(N_samples, 1) + 1.0 * sin(0.8 * t_sim_array); % Vento asse Z (m/s)

gust_matrix = [zeros(N_samples, 1), noise_v, noise_w]; 

% Raffica violenta programmata a t=20s
idx_raffica = (t_sim_array >= 20.0 & t_sim_array <= 22.0);
gust_matrix(idx_raffica, 3) = gust_matrix(idx_raffica, 3) + 20; % +20 m/s downdraft!

gust_ts = timeseries(gust_matrix, t_sim_array);

% Micro-correzioni "pilota" fantasma sugli alettoni (vibrazioni minime)
ail_noise = 0.2 * (pi/180) * randn(N_samples, 1); % 0.2 gradi di vibrazione
rud_noise = 0.1 * (pi/180) * randn(N_samples, 1); % 0.1 gradi di vibrazione

ail_ts   = timeseries(ail_noise, t_sim_array);
rud_ts   = timeseries(rud_noise, t_sim_array);

disp('ATTENZIONE: Turbolenza continua attivata! Vento discensionale estremo a t=20s!');

% Lancio di Simulink
simout = sim('F16_MPC', 'StopTime', num2str(Tend)); 

% Plot
plot_trajectories(simout);

    disp('Simulazione completata. Grafici generati.');
    disp(' ');
    disp('Avvio automatico di FlightGear in corso...');
    fg_cmd = 'open -a /Users/leonardoleggeri/Desktop/FlightGear.app --args --fg-aircraft=/Users/leonardoleggeri/Desktop/PROGETTI/Automazione/AutomationF16/Code/Studio_MIMO/F16MIMO/FlightGear/Aircraft --fdm=network,localhost,5501,5502,5503 --aircraft=f16 --airport=KSFO --timeofday=noon --disable-sound';
    system(fg_cmd);
    
    disp(' ');
    input('>>> Quando vedi che l''aereo è caricato sulla pista, premi INVIO qui per far partire il replay 3D! <<<');
    play_flightgear(simout);
