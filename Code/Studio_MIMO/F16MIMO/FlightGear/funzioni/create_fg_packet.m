function data = create_fg_packet(lon, lat, alt, phi, theta, psi)
    % Crea un pacchetto binario net_fdm per FlightGear (v24)
    % Formato Big-Endian (Network Byte Order)
    % Dimensione tipica del pacchetto struct FGNetFDM è 408 bytes
    
    data = zeros(1, 408, 'uint8');
    
    % uint32 version = 24
    data(1:4) = typecast(swapbytes(uint32(24)), 'uint8');
    
    % double longitude (rad)
    data(9:16) = typecast(swapbytes(double(lon)), 'uint8');
    
    % double latitude (rad)
    data(17:24) = typecast(swapbytes(double(lat)), 'uint8');
    
    % double altitude (m)
    data(25:32) = typecast(swapbytes(double(alt)), 'uint8');
    
    % float agl (m)
    data(33:36) = typecast(swapbytes(single(alt)), 'uint8');
    
    % float phi (rad) - roll
    data(37:40) = typecast(swapbytes(single(phi)), 'uint8');
    
    % float theta (rad) - pitch
    data(41:44) = typecast(swapbytes(single(theta)), 'uint8');
    
    % float psi (rad) - yaw
    data(45:48) = typecast(swapbytes(single(psi)), 'uint8');
end
