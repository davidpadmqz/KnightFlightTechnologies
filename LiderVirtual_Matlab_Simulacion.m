%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autores: Equipo Knight Flight Technologies
% Fecha de creación: Semana 4  
% Fecha de última modificación: Semana 5
% Descripción: Código para simular el vuelo de dos drones siguiendo al
% líder virtual
% Formato de nombrar variables y funciones: Minúsculas y descriptivas
% Comentado por: Carlos Flores
%Nota: Para un dron se elimina uno de los lados (con la finalidad de
%simplificar la lógica de vuelo)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%posicion lider virtual
x = 0;
y = 0;

%posicion dron 1
x_drone1 = -1;
y_drone1 = -1;

%posicion dron 2
x_drone2 = 0;
y_drone2 = -1;


%constantes de control para drones
kp_drone = 5;

l = 0.2; %Distancia entre drones

%Tiempos y relacionados
t = 0;
Tf = 0.1;
dt = 0.01;


theta =  0.7854; %Ángulo de inicio 


kt = 10;
kr = 100;


xd = (0:0.1:4); %vector de puntos deseados en x
yd = (sin(xd));% vector de puntos deseados en y usando función seno
[filas, columnas] = size(xd); 

figure(1);
plot(xd,yd); %Gráfica de la función seno a seguir
hold on

%distancias conocidas para la referencia geometrica
dc = 0.5;

for i = 1:columnas %ciclo for para recorrer cada punto deseado


   xdd = xd(i);
   ydd = yd(i);

   p1x = x - dc*sin(theta);
   p1y = y + dc*cos(theta);

   p2x = x + dc*sin(theta);
   p2y = y - dc*cos(theta);

   figure(1)
   scatter(p1x,p1y)
   scatter(p2x, p2y)

   t=0;

while (t<Tf) %ciclo while para llegar a cada punto deseado en la funcion seno

    xe = x - xdd;
    ye = y - ydd;

    thetad = atan2(ydd-y,xdd-x);
    thetae = theta - thetad;

    v = kt*sqrt(xe^2 + ye^2);
    omega = -kr*thetae;

    if v > 1
        v = 1;
    end
    if omega > pi/2
        omega = pi/2;
    end
    if omega < -pi/2
        omega = -pi/2;
    end

    vr = v + 0.5*l*omega;
    vl = v - 0.5*l*omega;

    v = (vr + vl)/2;
    omega = (vr - vl)/l;

    xp = v*cos(theta);
    yp = v*sin(theta);
    thetap = omega;

    %Resta para obtener velocidades del dron1
    vx_drone1 = (p1x - x_drone1)*kp_drone;
    vy_drone1 = (p1y - y_drone1)*kp_drone;

    %Resta para obtener velocidades del dron2
    vx_drone2 = (p2x - x_drone2)*kp_drone;
    vy_drone2 = (p2y - y_drone2)*kp_drone;

    %Integrar para obtener posicion del lider
    x = x + xp*dt;
    y = y + yp*dt ;
    
    %Integrar para obtener posicion del dron 1
    x_drone1 = x_drone1 + vx_drone1*dt;
    y_drone1 = y_drone1 + vy_drone1*dt;

    %Integrar para obtener posicion del dron 2
    x_drone2 = x_drone2 + vx_drone2*dt;
    y_drone2 = y_drone2 + vy_drone2*dt;
    

    theta = theta + thetap*dt;

    t = t + dt;

    %error_x = abs((xdd-x))/abs(xdd);
    %error_y = abs((ydd-y))/abs(ydd);


    figure(1)
    scatter(x,y)
    scatter(x_drone1, y_drone1)
    scatter(x_drone2, y_drone2)
    
end
end