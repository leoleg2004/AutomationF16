% Script finale per i 3 cavi in uscita
disp('--- Sto collegando le 3 uscite dell''MPC ---');
bdclose('all');
open_system('F16_MPC');

mpc_blk = 'F16_MPC/MPC_ClosedLoop_Controller';
mpc_ports = get_param(mpc_blk, 'PortHandles');

% Cerco l'aereo F-16
f16_blocks = find_system('F16_MPC', 'SearchDepth', 1, 'RegExp', 'on', 'Name', '.*F16.*Model.*');
f16_blk = f16_blocks{1};
f16_ports = get_param(f16_blk, 'PortHandles');

% 1. Collega Elevator (Porta 2 dell'MPC -> Porta 4 dell'aereo)
try
    add_line('F16_MPC', mpc_ports.Outport(2), f16_ports.Inport(4));
    disp('Elevator collegato!');
catch
end

% 2. Collega Flaps/LEF (Porta 3 dell'MPC -> Porta 7 dell'aereo)
try
    add_line('F16_MPC', mpc_ports.Outport(3), f16_ports.Inport(7));
    disp('Flaps (lef) collegato!');
catch
end

% 3. Collega Thrust al Mux
try
    % Trovo il blocco attaccato all'ingresso 2 dell'aereo (Fu)
    line_to_Fu = get_param(f16_ports.Inport(2), 'Line');
    mux_outport = get_param(line_to_Fu, 'SrcPortHandle');
    mux_blk = get_param(mux_outport, 'Parent');
    mux_ports = get_param(mux_blk, 'PortHandles');
    
    % Collego la porta 1 dell'MPC alla porta 1 del Mux
    add_line('F16_MPC', mpc_ports.Outport(1), mux_ports.Inport(1));
    disp('Thrust collegato al Mux!');
catch
end

save_system('F16_MPC');

disp(' ');
disp('====================================================');
disp(' FATTO! Cavi collegati con successo!');
disp('====================================================');
