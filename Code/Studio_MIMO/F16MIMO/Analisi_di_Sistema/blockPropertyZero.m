% =========================================================================
% VALUTAZIONE ZERI E PROPRIETÀ BLOCCANTE (Sistema Completo 3x3)
% Ing. Leggeri Leonardo
% =========================================================================
disp('--- Analisi Proprietà Bloccante su Sistema Completo 3x3 ---');

% Estraiamo le dimensioni e le sottomatrici
n_stati = size(A_long, 1);
ingressi_3x3 = [1, 2, 3]; % Thrust, Elevator, LE_Flap
uscite_3x3 = [1, 2, 3];   % V_t, alpha, q

B_3x3 = B_long(:, ingressi_3x3);
C_3x3 = C_long(uscite_3x3, :);
D_3x3 = D_long(uscite_3x3, ingressi_3x3);

% Modello in Spazio di Stato 3x3
sys_3x3 = ss(A_long, B_3x3, C_3x3, D_3x3);

% Calcolo degli zeri invarianti
Zeri_3x3 = tzero(sys_3x3);

disp('Zeri Invarianti calcolati per il sistema 3x3:');
disp(Zeri_3x3); % Sarà un array vuoto []

% Verifica e Spiegazione a schermo
if isempty(Zeri_3x3)
    disp('---------------------------------------------------------');
    disp('RISULTATO STRUTTURALE: Il sistema 3x3 NON ha zeri invarianti.');
    disp('Non esiste alcuna frequenza lambda (reale o complessa) capace');
    disp('di bloccare la trasmissione degli ingressi verso le uscite.');
    disp('La matrice di sistema ha sempre rango pieno.');
    disp('---------------------------------------------------------');
end

% =========================================================================
% CONTROPROVA: Proprietà Bloccante su Sottosistema Sottoattuato (2x2)
% =========================================================================
disp('--- Analisi Proprietà Bloccante Sottosistema 2x2 ---');

n_stati = size(A_long, 1);

% Selezioniamo 2 ingressi (Thrust, Elevator) e 2 uscite (V_t, q)
ingressi_2x2 = [1, 2]; 
uscite_2x2 = [1, 3];   

B_2x2 = B_long(:, ingressi_2x2);
C_2x2 = C_long(uscite_2x2, :);
D_2x2 = D_long(uscite_2x2, ingressi_2x2);

% Creazione Sottosistema
sys_2x2 = ss(A_long, B_2x2, C_2x2, D_2x2);

% Calcolo Zeri Invarianti
Zeri_2x2 = tzero(sys_2x2);
disp('Zeri Invarianti calcolati per il 2x2:');
disp(Zeri_2x2);

if ~isempty(Zeri_2x2)
    % Prende il primo zero (reale)
    lambda = Zeri_2x2(1); 
    
    % Matrice di Rosenbrock
    P_lambda = [lambda * eye(n_stati) - A_long, -B_2x2; 
                C_2x2,                           D_2x2];
    
    % Nullspace per trovare direzioni bloccanti
    N = null(P_lambda); 
    
    if ~isempty(N)
        % Estrazione direzioni (parte reale per sicurezza)
        x0 = real(N(1:n_stati, 1));
        u0 = real(N(n_stati+1:end, 1));
        
        % Vettore d'ingresso perturbato (direzione errata)
        u0_errato = u0;
        u0_errato(1) = -u0_errato(1); % Invertiamo un segno per rovinare la direzione
        
        % Vettore tempo
        t = 0:0.01:5; 
        
        % Ingressi nel tempo
        u_t_corretto = (u0 * exp(real(lambda) * t))'; 
        u_t_errato   = (u0_errato * exp(real(lambda) * t))'; 
        
        % ==========================================
        % CASO 1: Condizione Perfetta (Proprietà Bloccante Attiva)
        % ==========================================
        [y_sim_1, t_sim_1, x_sim_1] = lsim(sys_2x2, u_t_corretto, t, x0);
        
        % ==========================================
        % CASO 2: Stato Iniziale Nullo (x0 = 0)
        % ==========================================
        [y_sim_2, t_sim_2, x_sim_2] = lsim(sys_2x2, u_t_corretto, t, zeros(n_stati,1));
        
        % ==========================================
        % CASO 3: Direzione d'Ingresso Errata
        % ==========================================
        [y_sim_3, t_sim_3, x_sim_3] = lsim(sys_2x2, u_t_errato, t, x0);
        
        % ==========================================
        % GRAFICO FORMATTATO PER LA TESI (3 CASISTICHE)
        % ==========================================
        fig = figure('Name', 'Proprieta_Bloccante_2x2_Completa', 'Color', 'w', 'Position', [100, 100, 1000, 600]);
        
        % Limiti Y globali per confrontare equamente
        ymax = max([max(abs(y_sim_2(:))), max(abs(y_sim_3(:))), 1e-3]);
        
        % CASO 1
        subplot(1,3,1);
        plot(t, y_sim_1, 'LineWidth', 2);
        title({'CASO 1: Blocco Perfetto', 'x(0) = x_0, u(t) = u_0 e^{\lambda t}'}, 'FontSize', 10, 'FontWeight', 'bold');
        xlabel('Tempo [s]'); ylabel('Variazione Uscite (Y)');
        ylim([-ymax ymax]);
        grid on;
        legend('$V_t$', '$q$', 'Location', 'best', 'Interpreter', 'latex');
        
        % CASO 2
        subplot(1,3,2);
        plot(t, y_sim_2, 'LineWidth', 2);
        title({'CASO 2: Errore Stato Iniziale', 'x(0) = 0, u(t) = u_0 e^{\lambda t}'}, 'FontSize', 10, 'FontWeight', 'bold');
        xlabel('Tempo [s]');
        ylim([-ymax ymax]);
        grid on;
        
        % CASO 3
        subplot(1,3,3);
        plot(t, y_sim_3, 'LineWidth', 2);
        title({'CASO 3: Errore Direzione Input', 'x(0) = x_0, u(t) = u_{err} e^{\lambda t}'}, 'FontSize', 10, 'FontWeight', 'bold');
        xlabel('Tempo [s]');
        ylim([-ymax ymax]);
        grid on;
        
        sgtitle('Verifica della Proprietà Bloccante degli Zeri nel Sottosistema 2x2', 'FontSize', 14, 'FontWeight', 'bold');
        
        disp('Grafico generato con le 3 casistiche. Salvalo per la tesi!');
    else
        disp('Errore: Nullspace vuoto.');
    end
end