function stop = pswplot3D(optimValues, state)
    % Plot 3D/Contour personalizzato per Particle Swarm
    % Disegna le particelle nello spazio: Spinta (X), Equilibratore (Y), Errore (Z)
    % e crea una superficie (funzione di costo) interpolata basata sui punti esplorati
    persistent hScatter hBest hSurface storX storY storZ

    stop = false; 

    switch state
        case 'init'
            figure('Name', 'Mappatura Superficie di Costo', 'Color', 'w');
            colormap('jet'); 
            
            storX = []; storY = []; storZ = [];
            
            % Disegna la superficie vuota all'inizio
            hSurface = surf(NaN(2), NaN(2), NaN(2), 'EdgeColor', 'none', 'FaceAlpha', 0.4);
            hold on;
            
            % Disegna le particelle
            hScatter = scatter3(optimValues.swarm(:,1), optimValues.swarm(:,2), optimValues.swarmfvals, ...
                                50, 'k', 'filled');
            
            % Migliore particella
            hBest = scatter3(optimValues.bestx(1), optimValues.bestx(2), optimValues.bestfval, ...
                             350, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
            
            title('Superficie di Costo: Spinta vs Equilibratore vs Errore', 'FontSize', 14);
            xlabel('Spinta (lbs)', 'FontSize', 12);
            ylabel('Equilibratore (rad)', 'FontSize', 12);
            zlabel('Errore (Costo)', 'FontSize', 12);
            grid on;
            view(-30, 45);
            drawnow;
            
        case 'iter'
            if isgraphics(hScatter) && isgraphics(hBest)
                % Salva lo storico dei punti visitati per creare il "terreno"
                storX = [storX; optimValues.swarm(:,1)];
                storY = [storY; optimValues.swarm(:,2)];
                storZ = [storZ; optimValues.swarmfvals];
                
                % Aggiorna le particelle
                hScatter.XData = optimValues.swarm(:,1);
                hScatter.YData = optimValues.swarm(:,2);
                hScatter.ZData = optimValues.swarmfvals;
                
                % Aggiorna il Best
                hBest.XData = optimValues.bestx(1);
                hBest.YData = optimValues.bestx(2);
                hBest.ZData = optimValues.bestfval;
                
                % Aggiorna il terreno (Mesh / Contour) se abbiamo abbastanza punti
                if length(storX) > 10 && isgraphics(hSurface)
                    % Crea una griglia regolare
                    xq = linspace(min(storX), max(storX), 30);
                    yq = linspace(min(storY), max(storY), 30);
                    [Xq, Yq] = meshgrid(xq, yq);
                    
                    % Usa un try-catch nel caso griddata fallisca per punti collineari
                    try
                        Zq = griddata(storX, storY, storZ, Xq, Yq, 'natural');
                        hSurface.XData = Xq;
                        hSurface.YData = Yq;
                        hSurface.ZData = Zq;
                        hSurface.CData = Zq;
                    catch
                    end
                end
                
                drawnow;
            end
    end
end
