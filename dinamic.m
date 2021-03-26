function [qpp] = dinamic(q,qp,T_ref,F,l)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
q1=q(1);
q2=q(2);
qp1=qp(1);
qp2=qp(2);

J=jacobian(q,l);
R=Rot(q);
J_e=inv(R)*J;
M = [1.7277+0.1908*cos(q2) 0.0918+0.0954*cos(q2);...
        0.0918+0.0954*cos(q2) 0.9184];
   C = [31.8192-0.0954*sin(q2)*qp2 -0.0954*sin(q2)*(qp1+qp2);...
        0.3418*sin(q2)*qp1 12.5783];
   f = [1.0256*sign(qp1);...
        1.7842*sign(qp2)];
qpp = inv(M)*(T_ref-C*qp-f+J_e'*F); %Aceleraci�n articular de salida  
end

