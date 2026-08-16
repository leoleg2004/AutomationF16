% =========================================================================
% Tesi Triennale - Validazione MPC Stocastico su Simulink e FlightGear
% Ing. Leggeri Leonardo
% =========================================================================

% Svuota cache Simulink
bdclose('all');
clear F16_MPC;
rehash;

% 1. Prepara il file Simulink caricando i parametri e eseguendo l'MPC stocastico
disp('Avvio Simulazione Matematica dell''MPC Stocastico...');
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
startup_project();
conversion; 

% ESEGUE LO SCRIPT STOCASTICO CREATO DALL'UTENTE
main_mpc_closed_loop_stocastico;

disp('Preparazione della simulazione Open-Loop in Simulink per validare le azioni correttive stocastiche...');

% L'MPC stocastico ha generato le azioni "storia_u_stoc" in risposta al "storia_vento"
% Le iniettiamo come open-loop nel modello non-lineare!
storia_u = storia_u_stoc;

% Impostazione delle Condizioni Iniziali (IC) per Simulink
h0 = -4000 * ft2m; % Altitudine
IC.inertial_position = [0, 0, h0]; 

if exist('x_iniziale', 'var')
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

% Preparazione delle traiettorie Timeseries per Simulink
tempo_desiderato_fg = 40; % 40 secondi totali per FlightGear
step_desiderati = round(tempo_desiderato_fg / Ts);

if size(storia_u, 2) < step_desiderati
    pad_len = step_desiderati - size(storia_u, 2);
    ultimo_u = storia_u(:, end);
    storia_u_fg = [storia_u, repmat(ultimo_u, 1, pad_len)];
    
    ultimo_vento = zeros(size(storia_vento, 1), 1); % Vento cessa alla fine del calcolo matematico
    storia_vento_fg = [storia_vento, repmat(ultimo_vento, 1, pad_len)];
else
    storia_u_fg = storia_u;
    storia_vento_fg = storia_vento;
end

num_steps = size(storia_u_fg, 2);
sim_time = (0 : num_steps-1)' * Ts;

% L'MPC calcola perturbazioni, sommiamo i valori di trim
Thrust_val = best_u(1) + storia_u_fg(1, :)';
ele_val    = best_u(2) + storia_u_fg(2, :)'; % In radianti
dlef_val   = best_u(3) + storia_u_fg(3, :)'; % In radianti

% Creazione oggetti timeseries leggibili direttamente da "From Workspace"
Thrust_ts = timeseries(Thrust_val, sim_time);
ele_ts    = timeseries(ele_val, sim_time);
dlef_ts   = timeseries(dlef_val, sim_time);
ail_ts    = timeseries(zeros(num_steps, 1), sim_time);
rud_ts    = timeseries(zeros(num_steps, 1), sim_time);

% Mappiamo il vento stocastico (U e W) nel vettore 3D Gust (X, Y, Z)
% storia_vento(1,:) = perturbazione asse X (U)
% storia_vento(2,:) = perturbazione asse Z (W)
gust_matrix = [storia_vento_fg(1, :)', zeros(num_steps, 1), storia_vento_fg(2, :)'];
gust_ts   = timeseries(gust_matrix, sim_time);

disp('Traiettoria dei comandi stocastici e del vento generati nel workspace.');

% Esecuzione automatica in Simulink
Tend = max(sim_time);
disp(['Simulazione Open-Loop in Simulink per Tend = ', num2str(Tend), 's...']);
simout = sim('F16_MPC', 'StopTime', num2str(Tend)); 

% Plot finale Simulink
plot_trajectories(simout);

disp('Simulazione completata. Grafici generati.');
disp(' ');
disp('Avvio automatico di FlightGear in corso...');
fg_cmd = 'open -a /Users/leonardoleggeri/Desktop/FlightGear.app --args --fg-aircraft=/Users/leonardoleggeri/Desktop/PROGETTI/Automazione/AutomationF16/Code/Studio_MIMO/F16MIMO/FlightGear/Aircraft --fdm=network,localhost,5501,5502,5503 --aircraft=f16 --airport=KSFO --timeofday=noon --disable-sound';
system(fg_cmd);

disp(' ');
input('>>> Quando vedi che l''aereo è caricato sulla pista, premi INVIO qui per far partire il replay 3D! <<<');
play_flightgear(simout);
