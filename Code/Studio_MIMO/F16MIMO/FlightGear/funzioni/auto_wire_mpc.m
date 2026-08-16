% Script per automatizzare l'inserimento dell'MPC in Simulink!
% Esegui questo script dalla Command Window di MATLAB.

disp('Inizializzazione modifica automatica del modello F16_MPC.slx...');
bdclose('all');
open_system('F16_MPC');

disp('1. Cerco ed elimino i vecchi blocchi "From Workspace"...');
fw_blocks = find_system('F16_MPC', 'BlockType', 'FromWorkspace');

if length(fw_blocks) ~= 3
    warning(['Attenzione: ho trovato ' num2str(length(fw_blocks)) ' blocchi "From Workspace" anziché 3.']);
    disp('Sto eliminando quelli trovati...');
end

% Salvo le porte di destinazione (gli ingressi dell'aereo)
dst_ports = [];
for i = 1:length(fw_blocks)
    ports = get_param(fw_blocks{i}, 'PortConnectivity');
    if ~isempty(ports) && ~isempty(ports(1).DstPort)
        dst_ports = [dst_ports; ports(1).DstPort(1)];
    end
    % Elimina linea e blocco
    try
        line = get_param(fw_blocks{i}, 'LineHandles');
        delete_line(line.Outport(1));
    catch
    end
    delete_block(fw_blocks{i});
end

disp('2. Inserisco il blocco intelligente "S-Function" (il nuovo MPC)...');
sfunc_path = 'F16_MPC/MPC_ClosedLoop_Controller';
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', sfunc_path);
set_param(sfunc_path, 'FunctionName', 'mpc_sfunc');
set_param(sfunc_path, 'Position', [150, 150, 300, 250]);

disp('3. Tento il collegamento automatico delle uscite verso l''aereo...');
try
    sfunc_ports = get_param(sfunc_path, 'PortHandles');
    for i = 1:min(length(dst_ports), 3)
        add_line('F16_MPC', sfunc_ports.Outport(i), dst_ports(i), 'autorouting', 'on');
    end
    disp('Uscite (Thrust, Elevator, Flaps) collegate con successo!');
catch
    disp('Non sono riuscito a collegare le uscite. Dovrai collegarle manualmente.');
end

disp(' ');
disp('====================================================');
disp(' FATTO! Ho modificato il tuo Simulink.');
disp(' ');
disp(' ORA TOCCA A TE FARE L''ULTIMA COSA (Ci metti 5 secondi):');
disp(' 1. Vai in Simulink (dovrebbe essersi aperta la finestra).');
disp(' 2. Cerca il nuovo blocco verde chiaro "MPC_ClosedLoop_Controller".');
disp(' 3. Vedrai che ha 4 ingressi a sinistra scollegati.');
disp(' 4. Trascina 4 fili dalle uscite del tuo aereo F-16 a quegli ingressi.');
disp('    L''ordine deve essere: theta, q, U, W.');
disp(' 5. Salva il modello e lancia runF16_MPC_ClosedLoop!');
disp('====================================================');
