% Script per automatizzare i 4 COLLEGAMENTI DI INGRESSO finali!
% Esegui questo script dalla Command Window di MATLAB.

disp('--- Auto-collegamento degli ingressi F-16 -> MPC in corso ---');
bdclose('all');
open_system('F16_MPC');

% Trova l'aereo basandosi sul nome approssimativo (ignora i ritorni a capo)
f16_blocks = find_system('F16_MPC', 'SearchDepth', 1, 'RegExp', 'on', 'Name', '.*F16.*Model.*');
if isempty(f16_blocks)
    error('Errore: impossibile trovare il blocco F-16 Model nel modello.');
end
f16_blk = f16_blocks{1};

% Trova l'MPC cercando qualsiasi blocco che contenga il nome MPC_ClosedLoop_Controller
mpc_blocks = find_system('F16_MPC', 'SearchDepth', 1, 'RegExp', 'on', 'Name', '.*MPC_ClosedLoop_Controller.*');
if isempty(mpc_blocks)
    error('Errore: impossibile trovare il blocco con il nome "MPC_ClosedLoop_Controller". Assicurati che si chiami esattamente così.');
end
mpc_blk = mpc_blocks{1};

% Elimina eventuali linee parzialmente collegate dall'utente
disp('1. Pulisco i cavi di ingresso (se ce ne sono)...');
mpc_ports = get_param(mpc_blk, 'PortHandles');
for i = 1:4
    line = get_param(mpc_ports.Inport(i), 'Line');
    if line ~= -1
        delete_line(line);
    end
end

% Aggiunge selettori per estrarre theta(2) e q(2) dai vettori EA e om
disp('2. Creo i Selettori per estrarre theta e q dai vettori di volo...');
try delete_block('F16_MPC/Sel_theta'); catch; end
try delete_block('F16_MPC/Sel_q'); catch; end

add_block('simulink/Signal Routing/Selector', 'F16_MPC/Sel_theta', 'Position', [500 150 540 190]);
set_param('F16_MPC/Sel_theta', 'Indices', '2', 'InputPortWidth', '3');

add_block('simulink/Signal Routing/Selector', 'F16_MPC/Sel_q', 'Position', [500 250 540 290]);
set_param('F16_MPC/Sel_q', 'Indices', '2', 'InputPortWidth', '3');

% Collega l'aereo ai Selettori
disp('3. Collego l''aereo ai Selettori (theta e q)...');
f16_ports = get_param(f16_blk, 'PortHandles');
sel_theta_ports = get_param('F16_MPC/Sel_theta', 'PortHandles');
sel_q_ports = get_param('F16_MPC/Sel_q', 'PortHandles');

add_line('F16_MPC', f16_ports.Outport(6), sel_theta_ports.Inport(1), 'autorouting', 'smart'); % EA -> Sel_theta
add_line('F16_MPC', f16_ports.Outport(8), sel_q_ports.Inport(1), 'autorouting', 'smart');     % om -> Sel_q

% Collega Selettori e Aereo al MPC
disp('4. Collego tutto al nuovo controller MPC...');
add_line('F16_MPC', sel_theta_ports.Outport(1), mpc_ports.Inport(1), 'autorouting', 'smart'); % theta -> In1
add_line('F16_MPC', sel_q_ports.Outport(1), mpc_ports.Inport(2), 'autorouting', 'smart');     % q -> In2
add_line('F16_MPC', f16_ports.Outport(3), mpc_ports.Inport(3), 'autorouting', 'smart');       % Vt -> In3
add_line('F16_MPC', f16_ports.Outport(4), mpc_ports.Inport(4), 'autorouting', 'smart');       % alpha -> In4

save_system('F16_MPC');

disp(' ');
disp('====================================================');
disp(' FATTO! I 4 collegamenti sono stati completati da MATLAB.');
disp(' Il tuo Simulink è ora un gioiello in puro Anello Chiuso.');
disp(' Puoi lanciare:');
disp('   runF16_MPC_ClosedLoop');
disp('====================================================');
