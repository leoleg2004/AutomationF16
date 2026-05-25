% STARTUP.m
% Questo script viene eseguito automaticamente da MATLAB all'avvio 
% (se la cartella corrente è impostata su questa directory)
% oppure all'apertura del Progetto MATLAB.

disp('--- Esecuzione Automatica di Startup ---');
addpath(fullfile(pwd, 'Startup'));
startup_project();
