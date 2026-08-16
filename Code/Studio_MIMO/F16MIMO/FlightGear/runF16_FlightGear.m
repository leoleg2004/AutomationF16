% runF16_FlightGear.m
% Esegue la simulazione MPC e visualizza i risultati in FlightGear.


% 1. Esegui la simulazione Simulink originale
disp('Avvio Simulazione Matematica dell''MPC...');
runF16_MPC_ClosedLoop

% 2. Avvia FlightGear in background
disp('Avvio di FlightGear...');
fg_cmd = 'open -a /Users/leonardoleggeri/Desktop/FlightGear.app --args --fg-aircraft=/Users/leonardoleggeri/Desktop/PROGETTI/Automazione/AutomationF16/Code/Studio_MIMO/F16MIMO/FlightGear/Aircraft --fdm=network,localhost,5501,5502,5503 --aircraft=f16 --airport=KSFO --timeofday=noon --disable-sound';
system(fg_cmd);

disp('In attesa che FlightGear si carichi (45 secondi)... NON toccare nulla!');
pause(180);

% 3. Riproduce i dati in tempo reale su FlightGear!
play_flightgear(simout);
