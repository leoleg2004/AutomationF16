% =========================================================================
% Copyright (c) 2026 Ing. Leggeri Leonardo
% Tutti i diritti riservati.
%
% ATTENZIONE: Questo software e il relativo codice sorgente sono di proprietà 
% esclusiva dell'Ing. Leggeri Leonardo. È severamente vietata la copia, 
% la distribuzione, la modifica o la vendita a terzi senza l'esplicito 
% consenso scritto dell'autore. La distribuzione o la vendita non autorizzata 
% costituisce reato ed è perseguibile penalmente secondo le leggi vigenti.
% =========================================================================

% STARTUP.m
% Questo script viene eseguito automaticamente da MATLAB all'avvio 
% (se la cartella corrente è impostata su questa directory)
% oppure all'apertura del Progetto MATLAB.

disp('--- Esecuzione Automatica di Startup ---');
addpath(fullfile(pwd, 'Startup'));
startup_project();
