function x_dot = pendulum(t,x,u,g,l,m,b)
% Modello del pendolo nello spazio degli stati
% parametri della funzione (tempo, vettore degli stati, azione di
% controllo, accelerazione di gravità, lunghezza del braccio, massa del
% pendolo, coefficiente di attrito

x_dot= zeros(2,1);
% Equazioni di stato
x_dot(1)= x(2);
x_dot(2)= -(u/l*cos(x(1))+g/l*sin(x(1))+b/(m*l^2)*x(2));

end
