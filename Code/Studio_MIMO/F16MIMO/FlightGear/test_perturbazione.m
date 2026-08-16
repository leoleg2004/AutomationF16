% Script per testare l'MPC con una forte perturbazione iniziale visibile
disp('======================================================');
disp('   TEST MPC: RECUPERO DA ASSETTO FORTEMENTE PERTURBATO');
disp('======================================================');

% Assicuriamoci che l'MPC sia caricato
if ~exist('mpc_prob', 'var')
    MPC;
end

% Definiamo una perturbazione molto visibile (solo longitudinale, 
% visto che il nostro MPC controlla solo beccheggio e quota, non il rollio)

% 1. Errore di Theta (Beccheggio): +30 gradi col naso in sù!
theta_err = 30 * pi / 180; 

% 2. Errore di beccheggio (q): 0 rad/s
q_err = 0.2;

% 3. Errore di Velocità (U): -15 m/s (parte molto lento, l'MPC dovrà dare manetta)
U_err = -15;

% 4. Errore di Velocità Verticale (W): +10 m/s
W_err = 10;

% Creiamo il vettore di stato iniziale perturbato per l'MPC
x_iniziale = [theta_err; q_err; U_err; W_err];

% Puliamo la memoria della S-Function per sicurezza
clear mpc_sfunc;

disp('Vettore di perturbazione applicato:');
disp(['Theta err : +', num2str(theta_err * 180 / pi), ' gradi (Naso in SU)']);
disp(['U err     : ', num2str(U_err), ' m/s']);

disp('Lancio la simulazione in Simulink. Guarda FlightGear!');
% Lanciamo lo script di esecuzione
runF16_MPC_ClosedLoop;
