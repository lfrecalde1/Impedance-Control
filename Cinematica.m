clc,clear all,close all;
%% Iportar libreria necesaria
%import ETS2.*;
import ETS3.*;
%% parametro de la cadena cinematica
a1=1;
L1 = 0; L2 = -0.2337; L3 = 0.4318; L4 = 0.0203; L5 = 0.0837; L6 = 0.4318;
%% generacion de la matriz de tranformacion
%E=Rz('q1')*Tx(a1);
E3 = Tz(L1) * Rz('q1') * Ry('q2') * Ty(L2) * Tz(L3) * Ry('q3')* Tx(L4) * Ty(L5) * Tz(L6) * Rz('q4') * Ry('q5') * Rz('q6');
cinematica=E3.fkine( [0, 0 0 0 45 90], 'deg')
R=[cinematica.n,cinematica.o,cinematica.a]
eulZYX = rotm2eul(R) %% usar este cambio de orientaciones
E3.teach;
axis([-1 1 -1 1 -1 1])
E3.plot([0, 0 0 0 45 90], 'deg')