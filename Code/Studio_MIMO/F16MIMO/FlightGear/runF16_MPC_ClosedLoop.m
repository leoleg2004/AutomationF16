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
Tend = 15; % secondi 
disp(['Simulazione Closed-Loop in Simulink per Tend = ', num2str(Tend), 's...']);

% Creazione segnali "fittizi" a zero per le costanti laterali e disturbi
sim_time = [0, Tend];
ail_ts   = timeseries([0; 0], sim_time);
rud_ts   = timeseries([0; 0], sim_time);
gust_ts  = timeseries([0 0 0; 0 0 0], sim_time);

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
