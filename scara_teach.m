%% generacion de un robot scara a partir de matrices de trasnformacion
clc,clear all,close all;
%% definicon de las variables simbolicas del sistema
syms q1 q2 q3 q4 as real;
l_1=1;
l_2=0.5;
l_3=0.3;
l_4=0.1;
%% crear el robot
L(1)=Link('revolute','d',l_1,'a',l_2,'alpha',0);
L(2)=Link('revolute','d',0,'a',l_3,'alpha',0);
L(3)=Link('prismatic','theta',0,'a',0,'alpha',0);
L(4)=Link('revolute','d',-l_4,'a',0,'alpha',pi);
robot = SerialLink( L,'name', 'my robot')