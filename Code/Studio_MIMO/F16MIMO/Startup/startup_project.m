function startup_project()
    % Aggiunge tutte le cartelle necessarie al path di MATLAB per 
    % garantire il funzionamento di tutti gli script e modelli.
    cartelle = {'funzioni_mpc', 'funzioni_lqr', 'Modello_Matematico', ...
                'Modelli_Simulink', 'Dati_Mat', 'Analisi_di_Sistema', ...
                'Studi_Avanzati_Invarianza', 'Simulazioni_Simulink', 'Utility_Extra', 'Startup'};
    
    for i = 1:length(cartelle)
        path_cartella = fullfile(pwd, cartelle{i});
        if exist(path_cartella, 'dir')
            addpath(path_cartella);
        end
    end
    disp('Path di progetto aggiornato con successo. Tutte le funzioni sono pronte.');
end
