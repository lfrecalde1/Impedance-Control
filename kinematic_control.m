function [qpref] = kinematic_control(hd,hdp,h,q,K1,K2,l)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
he=hd-h;
J=jacobian(q,l);

qpref=inv(J)*(hdp+K2*tanh(inv(K2)*K1*he));
end

