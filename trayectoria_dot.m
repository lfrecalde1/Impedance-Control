function [hxp,hyp] = trayectoria_dot(qd,qdp,l)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
for k=1:length(qd)
    J=jacobian(qd(:,k),l);
    hp = J*qdp(:,k);
    hxp(k)=hp(1);
    hyp(k)=hp(2);
end
end

