clear;
clc,
close all;

%1. Parametri del doppio serbatoio

%coefficiente di scarico

k=0.05;

%condizione iniziale 

h1_ini=0.7;%m
h2_ini=0.6;%m

%livello di riferimento (desiderato)Stati
h1_ref=1;%m
h2_ref=1.2;%m

%tempo di campionamento

Ts=0.1;%s


%tempo di simulazione

T_sim=20;

%vincoli di ingresso

u1_min=0;
u1_max=0.9;
u2_min=0;
u2_max=0.9;


%punto di equilibrio
u1_ref=k*sqrt(h2_ref);
u2_ref=u1_ref;

%creazione del modello linearizzato(State Space)



A = [0 0; 0 -k/(2*sqrt(h2_ref))]; % Matrice di stato
B = [1 -1; 0 1]; % Matrice di ingresso
C = eye(2); % Matrice di uscita eye fa la matrice identità
D = zeros(2,2); % Matrice di trasmissione diretta
sys = ss(A, B, C, D); % Creazione del sistema in spazio degli stati



sysD=c2d(sys,Ts,'zoh');%zero order hold o senno il metodo di eulero avanti

G=tf(sysD);


%%
 
% Creazione dei disaccopiatori


M12=-G(1,2)*G(2,2)^-1;
M11=1;
M21=0;
M22=M11;

M=[M11,M12;M21,M22];


%%
% modello simulink

model='exe12.slx';

%%configurazione primo controllore

C1=1;
C2=1;

% Simulazione del sistema
output_1=sim(model);

%plot dei risultati

figure
plot(out.h1_log, 'DisplayName', '$h_1$ [m]');
hold on;
plot(out.h2_log, 'DisplayName', '$h_2$ [m]');
yline(h1_ref, 'LabelHorizontalAlignment', 'left');
xlabel('Time [s]');
ylabel('Water Level [m]');
title('Water Levels in Double Tank System');
legend('show');
grid on;

figure
% Plot the control signals
figure
subplot(2,1,1);
stairs(out.u1_log_unbound.Time,...
    out.u1_log_unbound.Time);
plot(output_1.u1_log, 'DisplayName', 'Control Input $u_1$');
xlabel('Time [s]');
ylabel('Control Input [m]');
title('Control Input for Tank 1');
legend('show');
grid on;

figure
% Plot the control signals
figure
subplot(2,1,2);
stairs(out.u1_log_unbound.Time,...
    );
plot(output_1.u1_log, 'DisplayName', 'Control Input $u_1$');
xlabel('Time [s]');
ylabel('Control Input [m]');
title('Control Input for Tank 1');
legend('show');
grid on;





