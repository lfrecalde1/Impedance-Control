function [hxp,hyp] = direct_kinematic_dot(qp,q,l)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

J=jacobian(q,l);
hp = J*qp;
hxp=hp(1);
hyp=hp(2);

end

