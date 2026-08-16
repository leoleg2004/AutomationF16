% Script per forzare il cablaggio in Simulink (distrugge le vecchie linee ed evita le linee appese)
disp('--- Pulizia e Ricablaggio Forzato Uscite MPC ---');
bdclose('all');
open_system('F16_MPC');

f16_blocks = find_system('F16_MPC', 'SearchDepth', 1, 'RegExp', 'on', 'Name', '.*F16.*Model.*');
f16_blk = f16_blocks{1};
f16_ports = get_param(f16_blk, 'PortHandles');

mpc_blk = 'F16_MPC/MPC_ClosedLoop_Controller';
mpc_ports = get_param(mpc_blk, 'PortHandles');

% Funzione helper per eliminare la linea attaccata a una porta
function delete_line_if_exists(port_handle)
    l = get_param(port_handle, 'Line');
    if l ~= -1
        delete_line(l);
    end
end

% Trovo il MUX attaccato alla porta 2 dell'F-16
line_to_Fu = get_param(f16_ports.Inport(2), 'Line');
if line_to_Fu ~= -1
    mux_outport = get_param(line_to_Fu, 'SrcPortHandle');
    mux_blk = get_param(mux_outport, 'Parent');
    mux_ports = get_param(mux_blk, 'PortHandles');
else
    error('Mux per Fu non trovato!');
end

% 1. Pulisco tutte le vecchie linee in ingresso
delete_line_if_exists(f16_ports.Inport(4)); % ele
delete_line_if_exists(f16_ports.Inport(7)); % lef
delete_line_if_exists(mux_ports.Inport(1)); % Mux input 1 (Thrust)

% 2. Pulisco tutte le vecchie linee in uscita dall'MPC
delete_line_if_exists(mpc_ports.Outport(1));
delete_line_if_exists(mpc_ports.Outport(2));
delete_line_if_exists(mpc_ports.Outport(3));

% 3. CREO I NUOVI COLLEGAMENTI DIRETTI (senza autorouting per evitare crash)
add_line('F16_MPC', mpc_ports.Outport(1), mux_ports.Inport(1));
disp('Thrust cablato al Mux.');

add_line('F16_MPC', mpc_ports.Outport(2), f16_ports.Inport(4));
disp('Elevator cablato all''F-16.');

add_line('F16_MPC', mpc_ports.Outport(3), f16_ports.Inport(7));
disp('LEF/Flaps cablato all''F-16.');

save_system('F16_MPC');

disp('====================================================');
disp(' FATTO! Cavi ricostruiti da zero. Nessun cavo appeso.');
disp('====================================================');
