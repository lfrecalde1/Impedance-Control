function [J] = jacobian(q,l)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Datos de constantes del sistema
l_2=l(1);
l_3=l(2);

% estados internos del robot
q1=q(1);
q2=q(2);

J11=- l_3*sin(q1 + q2) - l_2*sin(q1);
J12=-l_3*sin(q1 + q2);

J21= l_3*cos(q1 + q2) + l_2*cos(q1);
J22= l_3*cos(q1 + q2);

J=[J11,J12;...
   J21,J22];
det(J);
end

