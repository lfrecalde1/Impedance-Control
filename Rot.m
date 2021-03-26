function [R] = Rot(q)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here
q1=q(1);
q2=q(2);
R11=cos(q1 + q2);
R12=sin(q1 + q2 );
R13=0;

R21=sin(q1 + q2 );
R22=-cos(q1 + q2);
R23=0;

R31=0;
R32=0;
R33=-1;
R=[R11,R12;...
    R21,R22];
end

