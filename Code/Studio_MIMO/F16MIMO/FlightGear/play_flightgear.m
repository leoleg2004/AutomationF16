function play_flightgear(simout)
    % Trasmette la simulazione salvata a FlightGear in tempo reale!
    % ZERO TOOLBOX RICHIESTI! Usa la libreria base di MATLAB.
    
    disp('Estrazione dati...');
    
    % Compatibilità con Dataset o Structure
    if isa(simout.yout, 'Simulink.SimulationData.Dataset')
        yout_temp = [];
        for i = 1:simout.yout.numElements
            yout_temp = horzcat(yout_temp, simout.yout{i}.Values.Data);
        end
        data = yout_temp;
        t = simout.tout; % usa tout generale della simulazione
    elseif isstruct(simout.yout) && isfield(simout.yout, 'signals')
        t = simout.yout.time;
        data = horzcat(simout.yout.signals.values);
    end
    
    % Crea la porta UDP (funzione base di MATLAB)
    try
        u = udpport('IPV4', 'LocalPort', 5500);
    catch
        % Fallback per versioni vecchie
        u = udp('localhost', 5502, 'LocalPort', 5500);
        fopen(u);
    end
    
    disp('Connessione UDP a FlightGear (porta 5502) stabilita!');
    disp('Inizio playback 3D in tempo reale...');
    
    % KSFO (San Francisco)
    lat0 = 37.6189 * pi/180;
    lon0 = -122.3750 * pi/180;
    alt0 = 10; % metri (quota partenza)
    
    R = 6371000; % raggio terra
    
    tic;    %% Interpolazione a 60 FPS per fluidità
    fps = 60;
    dt_new = 1/fps;
    t_new = t(1):dt_new:t(end);
    
    % Interpola la posizione e gli angoli
    data_new = interp1(t, data, t_new, 'pchip');
    
    disp(['Playback a ' num2str(fps) ' FPS per la massima fluidità...']);
    
    %% Loop di invio a 60 FPS
    for i = 1:length(t_new)
        % Estrai posizione inerziale (North, East, Down)
        Xe = data_new(i, 1:3); 
        
        % SBLOCCO ASSE LATERALE PER MPC LONGITUDINALE
        % Xe(2) = 0; % Permettiamo la deriva est
        
        % Angoli di Eulero (Phi, Theta, Psi)
        EA = data_new(i, 10:12);
        % EA(1) = 0; % Permettiamo il rollio naturale
        % EA(3) = 0; % Permettiamo l'imbardata naturale
        
        R = 6371000; % Raggio terrestre (m)
        dLat = Xe(1) / R;
        dLon = Xe(2) / (R * cos(lat0));
        
        lat = lat0 + dLat;
        lon = lon0 + dLon;
        
        % BLOCCO ALTITUDINE
        % L'MPC regola le velocità (U, W) ma non la quota.
        % Siccome l'angolo di rampa (gamma) è negativo, l'aereo scende e si schianta.
        % Per vedere il controllo del beccheggio senza cadere, blocchiamo la quota!
        alt = 1500; % Quota fissa a 1500 metri (~5000 ft)
        
        % Crea il pacchetto binario magico (Roll=EA(1)=0, Pitch=EA(2), Yaw=EA(3)=0)
        pkt = create_fg_packet(lon, lat, alt, EA(1), EA(2), EA(3));
        
        % Invia
        try
            write(u, pkt, "uint8", "127.0.0.1", 5502);
        catch
            fwrite(u, pkt, 'uint8');
        end
        
        % Sincronizzazione precisa a 60 FPS
        if i < length(t_new)
            pause(dt_new);
        end
    end
    
    disp('Playback concluso!');
    
    try clear u; catch; fclose(u); delete(u); end
end
