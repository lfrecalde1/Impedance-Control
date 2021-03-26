function [hx,hy] = trayectoria(qd,l)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
for k=1:length(qd)
    [hx(k),hy(k)] = direct_kinematic(qd(:,k),l);
end
end


