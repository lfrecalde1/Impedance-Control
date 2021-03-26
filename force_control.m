function [u] = force_control(qp,q,hdpp,hdp,hd,hp,h,Mm,Dm,Km,l,F)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
q1=q(1);
q2=q(2);
qp1=qp(1);
qp2=qp(2);
%% generacion del jacobiano
J=jacobian(q,l);

Jp=jacobian_dot(qp,q,l);

%% generacion de los errores
he=hd-h;
hep=hdp-hp;


R=Rot(q);
J_e=inv(R)*J;
%% generacion de la matrices de dinamica
M = [1.7277+0.1908*cos(q2) 0.0918+0.0954*cos(q2);...
    0.0918+0.0954*cos(q2) 0.9184];
C = [31.8192-0.0954*sin(q2)*qp2 -0.0954*sin(q2)*(qp1+qp2);...
    0.3418*sin(q2)*qp1 12.5783];
f = [1.0256*sign(qp1);...
    1.7842*sign(qp2)];
 u=M*inv(J)*(hdpp-Jp*qp+inv(Mm)*(Dm*hep+Km*he))+C*qp+f+(M*inv(J)*inv(Mm)*inv(R)-J_e')*F;

% u=M*inv(J)*(hdpp-Jp*qp)+J'*(Dm*hep+Km*he)+C*qp+f

% u=M*inv(J)*(hdpp-Jp*inv(J)*hdp)+C*inv(J)*hdp+f+J'*(Dm*hep+Km*he);
% u=M*inv(J)*(hdpp-Jp*qp+inv(Mm)*(Dm*hep+Km*he))+C*qp+f;
end

