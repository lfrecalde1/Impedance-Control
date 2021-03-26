function [hx,hy] = direct_kinematic(q,l)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

% Datos de constantes del sistema
l_2=l(1);
l_3=l(2);

% estados internos del robot
q1=q(1);
q2=q(2);

hx = l_3*cos(q1 + q2) + l_2*cos(q1);
hy = l_3*sin(q1 + q2) + l_2*sin(q1);
end

