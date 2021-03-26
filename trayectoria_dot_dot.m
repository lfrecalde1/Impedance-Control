function [hxpp,hypp] = trayectoria_dot_dot(qdpp,qdp,qd,l)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
for k=1:length(qd)
    J=jacobian(qd(:,k),l);
    Jp=jacobian_dot(qdp(:,k),qd(:,k),l);
    hpp = J*qdpp(:,k)+Jp*qdp(:,k);
    hxpp(k)=hpp(1);
    hypp(k)=hpp(2);
end
end

