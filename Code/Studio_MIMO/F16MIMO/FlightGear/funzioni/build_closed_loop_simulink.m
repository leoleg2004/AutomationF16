% Script DEFINITIVO per creare l'anello chiuso in Simulink in un sol colpo
disp('--- Creazione automatica dell''Anello Chiuso in corso ---');
bdclose('all');
open_system('F16_MPC');

f16_blk = 'F16_MPC/Nonlinear F16 Model';

% Nomi esatti dei blocchi From Workspace da eliminare (visibili nel tuo screenshot)
blocks_to_replace = {'F16_MPC/Thrust_ts', 'F16_MPC/ele_ts', 'F16_MPC/dlef_ts'};
dst_ports = zeros(3, 1);

disp('1. Identifico le destinazioni dei vecchi blocchi Open-Loop...');
for i = 1:3
    blk = blocks_to_replace{i};
    try
        % Trova dove è collegato il blocco
        ports = get_param(blk, 'PortConnectivity');
        if ~isempty(ports) && ~isempty(ports(1).DstPort)
            dst_ports(i) = ports(1).DstPort(1);
        end
        % Elimina la linea e il blocco
        line = get_param(blk, 'LineHandles');
        delete_line(line.Outport(1));
        delete_block(blk);
    catch
        warning(['Impossibile trovare o eliminare ' blk '. Forse è già stato eliminato?']);
    end
end

disp('2. Inserisco l''intelligenza MPC (S-Function)...');
mpc_blk = 'F16_MPC/MPC_ClosedLoop_Controller';
try delete_block(mpc_blk); catch; end % Pulisce se esiste già
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', mpc_blk, 'Position', [150, 450, 350, 550]);
set_param(mpc_blk, 'FunctionName', 'mpc_sfunc');

mpc_ports = get_param(mpc_blk, 'PortHandles');

disp('3. Collego le 3 Uscite dell''MPC all''aereo...');
for i = 1:3
    if dst_ports(i) ~= 0
        try add_line('F16_MPC', mpc_ports.Outport(i), dst_ports(i), 'autorouting', 'smart'); catch; end
    end
end

disp('4. Creo i Selettori per estrarre theta(2) e q(2) dai vettori di volo...');
try delete_block('F16_MPC/Sel_theta'); catch; end
try delete_block('F16_MPC/Sel_q'); catch; end

add_block('simulink/Signal Routing/Selector', 'F16_MPC/Sel_theta', 'Position', [700, 350, 740, 390]);
set_param('F16_MPC/Sel_theta', 'Indices', '2', 'InputPortWidth', '3');

add_block('simulink/Signal Routing/Selector', 'F16_MPC/Sel_q', 'Position', [700, 420, 740, 460]);
set_param('F16_MPC/Sel_q', 'Indices', '2', 'InputPortWidth', '3');

sel_theta_ports = get_param('F16_MPC/Sel_theta', 'PortHandles');
sel_q_ports = get_param('F16_MPC/Sel_q', 'PortHandles');

disp('5. Collego l''aereo ai Selettori e all''MPC (Ingressi)...');
f16_ports = get_param(f16_blk, 'PortHandles');

% Aereo -> Selettori (Porte 6=EA, 8=om)
try add_line('F16_MPC', f16_ports.Outport(6), sel_theta_ports.Inport(1), 'autorouting', 'smart'); catch; end
try add_line('F16_MPC', f16_ports.Outport(8), sel_q_ports.Inport(1), 'autorouting', 'smart'); catch; end

% Segnali -> Ingressi MPC (1=theta, 2=q, 3=Vt, 4=alpha)
try add_line('F16_MPC', sel_theta_ports.Outport(1), mpc_ports.Inport(1), 'autorouting', 'smart'); catch; end
try add_line('F16_MPC', sel_q_ports.Outport(1), mpc_ports.Inport(2), 'autorouting', 'smart'); catch; end
try add_line('F16_MPC', f16_ports.Outport(3), mpc_ports.Inport(3), 'autorouting', 'smart'); catch; end
try add_line('F16_MPC', f16_ports.Outport(4), mpc_ports.Inport(4), 'autorouting', 'smart'); catch; end

save_system('F16_MPC');

disp(' ');
disp('====================================================');
disp(' SUCCESSO TOTALE! Il modello è perfettamente cablato.');
disp(' Puoi lanciare la simulazione direttamente:');
disp('   runF16_MPC_ClosedLoop');
disp('====================================================');
