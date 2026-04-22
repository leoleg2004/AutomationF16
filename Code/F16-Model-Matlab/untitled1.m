clear;
clc;
close all;


%% 1. Parametri del doppio serbatoio  (con % faccio commento, con %% faccio sezione di codice)

% Coefficiente di scarico
k=0.05;

% Condizione iniziale
h1_ini = 0.7; % m
h2_ini = 0.6; % m

% Livello di riferimento (desiderato) - stati
h1_ref = 1;
h2_ref = 1.2;

% Tempo di campionamento
Ts = 0.1;

% Tempo di simulazione
T_sim = 20;

% Vincoli di ingresso
u1_min = 0;
u1_max = 0.9;
u2_min = 0;
u2_max = 0.9;

% Punto di equilibrio - ingressi
u1_ref = k*sqrt(h2_ref);
u2_ref = u1_ref;

%% 2. Creazione del modello linearizzato (state-space)
A = [0 0; 0 -k/(2*sqrt(h2_ref))];
B = [1 -1; 0 1];
C = eye(2);  %crea una matrice identità due per due
D = zeros(2, 2); %crea una matrice di qualsiasi dimensione di zeri

%% 3. Calcolo della Fdt a tempo continuo e a tempo discreto
s = tf('s'); %Questa variabile s è come se fosse z della trasformata z

G_ct = C*(s*eye(2) - A) \ B; %backslash fa l'inversa

G = c2d(G_ct, Ts); %G sarà la funzione di trasferimento a tempo discreto


%% 4. Progettazione dei disaccoppiatori
M12 = -G(1, 2)*G(2, 2)^(-1);
M11 = 1;
M21 = 0;
M22 = M11;

M= [M11 M22; M21 M22];

%% 5. Modello simulink
model = 'untitled.slx';

%% Configurazione 1 controllore
C1 = 1;
C2= 1;

output_1 = sim(model);

% Plot dei risultati
figure
plot(output_1.h1_log, 'DisplayName', '$h_1$ [m]');
hold on;
plot (output_1.h2_log, 'DisplayName', '$h_2$ [m]');
yline(h1_ref, 'Label', '$h_{1, ref}$', 'HandleVisibility', 'off');
yline(h2_ref, 'Label', '$h_{1, ref}$', 'HandleVisibility', 'off');

xlabel('Time [s]');
ylabel('Height [m]');
title('Tank Levels Over Time');
legend show;
grid on;

figure
subplot(2, 1, 1)
stairs(output_1.u1_log_unbound.Time, output_1.u1_log_unbound.Data, 'DisplayName', 'Computed control action')
title('Azione di controllo pompa 1');

figure
subplot(2, 1, 1)
stairs(output_1.u2_log_unbound.Time, output_1.u2_log_unbound.Data, 'DisplayName', 'Computed control action')
title('Azione di controllo pompa 2');

ylabel('Input [m/s]');
grid on;
legend show;


