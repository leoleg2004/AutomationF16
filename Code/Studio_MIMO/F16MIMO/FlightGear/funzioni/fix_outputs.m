% Script definitivo per collegare le uscite dell'MPC
disp('--- Ricollegamento uscite MPC in corso ---');
bdclose('all');
open_system('F16_MPC');

mpc_blk = 'F16_MPC/MPC_ClosedLoop_Controller';
mpc_ports = get_param(mpc_blk, 'PortHandles');

% 1. Collegamento Thrust
try
    thrust_ports = get_param('F16_MPC/Thrust', 'PortConnectivity');
    dst = thrust_ports(1).DstPort(1);
    delete_line(get_param('F16_MPC/Thrust', 'LineHandles').Outport(1));
    delete_block('F16_MPC/Thrust');
    add_line('F16_MPC', mpc_ports.Outport(1), dst, 'autorouting', 'smart');
    disp('Thrust collegato!');
catch
    disp('Errore nel collegare Thrust. Forse è già stato modificato?');
end

% 2. Collegamento Elevator
try
    ele_ports = get_param('F16_MPC/ele', 'PortConnectivity');
    dst = ele_ports(1).DstPort(1);
    delete_line(get_param('F16_MPC/ele', 'LineHandles').Outport(1));
    delete_block('F16_MPC/ele');
    add_line('F16_MPC', mpc_ports.Outport(2), dst, 'autorouting', 'smart');
    disp('Elevator collegato!');
catch
    disp('Errore nel collegare Elevator. Forse è già stato modificato?');
end

% 3. Collegamento Flaps/LEF
try
    lef_ports = get_param('F16_MPC/lef', 'PortConnectivity');
    dst = lef_ports(1).DstPort(1);
    delete_line(get_param('F16_MPC/lef', 'LineHandles').Outport(1));
    delete_block('F16_MPC/lef');
    add_line('F16_MPC', mpc_ports.Outport(3), dst, 'autorouting', 'smart');
    disp('Flaps (lef) collegato!');
catch
    disp('Errore nel collegare Flaps. Forse è già stato modificato?');
end

save_system('F16_MPC');

disp(' ');
disp('====================================================');
disp(' FATTO! Azioni di controllo collegate ai motori.');
disp(' Ora lancia runF16_MPC_ClosedLoop');
disp('====================================================');
