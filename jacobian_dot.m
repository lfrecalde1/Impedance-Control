function [J_dot] = jacobian_dot(qp,q,l)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
l_2=l(1);
l_3=l(2);

q1=q(1);
q2=q(2);

q1p=qp(1);
q2p=qp(2);

J11=- q1p*(l_3*cos(q1 + q2) + l_2*cos(q1)) - l_3*q2p*cos(q1 + q2);
J12=-l_3*cos(q1 + q2)*(q1p + q2p);
J21=- q1p*(l_3*sin(q1 + q2) + l_2*sin(q1)) - l_3*q2p*sin(q1 + q2);
J22=-l_3*sin(q1 + q2)*(q1p + q2p);

J_dot=[J11,J12;...
    J21,J22];
end

