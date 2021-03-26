function [q1,q2] = integracion(q,qp,To)
%UNTITLED6 Summary of this function goes here
q=q+qp*To;
q1=q(1);
q2=q(2);
end

