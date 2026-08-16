function tabs = crea_dashboard_mpc()
    % CREA_DASHBOARD_MPC Crea o recupera una singola finestra per tutti i plot
    % Restituisce una struct con i riferimenti a tutti i tab disponibili
    
    fig_name = 'MPC Dashboard F-16';
    fig = findobj('Type', 'figure', 'Name', fig_name);
    
    if isempty(fig)
        fig = figure('Name', fig_name, 'NumberTitle', 'off', 'Position', [100 100 1200 800], 'Color', 'w');
        tg = uitabgroup(fig, 'Tag', 'Dashboard_Tabs');
        
        % Creazione dei tab nominali
        tabs.stati     = uitab(tg, 'Title', 'Stati (Nominale)');
        tabs.attuatori = uitab(tg, 'Title', 'Attuatori (Nominale)');
        tabs.cis_3d    = uitab(tg, 'Title', 'Control Invariant Set 3D');
        tabs.cost_3d   = uitab(tg, 'Title', 'Funzione di Costo 3D');
        
        % Creazione dei tab stocastici
        tabs.stoc_stati     = uitab(tg, 'Title', 'Robustezza: Stati');
        tabs.stoc_attuatori = uitab(tg, 'Title', 'Robustezza: Attuatori');
        tabs.stoc_cis_3d    = uitab(tg, 'Title', 'Robustezza: CIS 3D');
        
        % Salviamo i riferimenti nella userdata del tabgroup per recuperarli
        tg.UserData = tabs;
    else
        % Recuperiamo i riferimenti se la finestra esiste già
        tg = findobj(fig, 'Tag', 'Dashboard_Tabs');
        if isempty(tg)
            clf(fig);
            tg = uitabgroup(fig, 'Tag', 'Dashboard_Tabs');
            
            tabs.stati     = uitab(tg, 'Title', 'Stati (Nominale)');
            tabs.attuatori = uitab(tg, 'Title', 'Attuatori (Nominale)');
            tabs.cis_3d    = uitab(tg, 'Title', 'Control Invariant Set 3D');
            tabs.cost_3d   = uitab(tg, 'Title', 'Funzione di Costo 3D');
            
            tabs.stoc_stati     = uitab(tg, 'Title', 'Robustezza: Stati');
            tabs.stoc_attuatori = uitab(tg, 'Title', 'Robustezza: Attuatori');
            tabs.stoc_cis_3d    = uitab(tg, 'Title', 'Robustezza: CIS 3D');
            tg.UserData = tabs;
        else
            tabs = tg.UserData;
            % Se l'utente ha una vecchia versione senza stoc_cis_3d, la aggiungiamo al volo
            if ~isfield(tabs, 'stoc_cis_3d')
                tabs.stoc_cis_3d = uitab(tg, 'Title', 'Robustezza: CIS 3D');
                tg.UserData = tabs;
            end
        end
        figure(fig); % Portiamo in primo piano
    end
end
