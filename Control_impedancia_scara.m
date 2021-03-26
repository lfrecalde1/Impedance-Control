%%------------------------Programa para controlador impedancia-----%%
%% Limpiar el espacio de trabajo
clc,clear all,close all;

%% Generacion del tiempo de simulacion
Tfin=100;
To=0.02;
t=[0:To:Tfin];

%% Constantes del manipulador
l_2=0.4;
l_3=0.3;

l=[l_2,l_3]';

%% Valores iniciales de las articulaciones
q1(1)=0*pi/180;
q2(1)=15*pi/180;

q1p(1)=0;
q2p(1)=0;

q=[q1(1) q2(1)]';
qp=[q1p(1) q2p(1)]';

%% Cinematica directa del sistema
[hx(1),hy(1)] = direct_kinematic(q,l);
[hxp(1),hyp(1)] = direct_kinematic_dot(qp,q,l);

%% Ganancia del controlador
Mm=1*eye(2);
Dm=2*eye(2);
Km=0.5*eye(2);

%% Trayectoria deseada del sistema
q1d = 0*pi/180*ones(1,length(t));   % Posici�n deseadas de la articular del eslab�n 1
q2d = 85*pi/180*ones(1,length(t));  % Posici�n deseadas de la articular del eslab�n 2

q1dp = 0*pi/180*ones(1,length(t));   % Posici�n deseadas de la articular del eslab�n 1
q2dp = 0*pi/180*ones(1,length(t));   % Posici�n deseadas de la articular del eslab�n 2

q1dpp =0*pi/180*ones(1,length(t));   % Posici�n deseadas de la articular del eslab�n 1
q2dpp =0*pi/180*ones(1,length(t));   % Posici�n deseadas de la articular del eslab�n 2

qd=[q1d;q2d];
qdp=[q1dp;q2dp];
qdpp=[q1dpp;q2dpp];

[hxd,hyd] = trayectoria(qd,l);
[hxdp,hydp] = trayectoria_dot(qd,qdp,l);
[hxdpp,hydpp] = trayectoria_dot_dot(qdpp,qdp,qd,l);

%% Bucle de control
%% Generar fuerzas de interaccion con el entorno
Fx=0.0*sin(0.1*t);
Fy=0*ones(1,length(t));
Fz=0*ones(1,length(t));
for k=1:length(t)
   if t(k)>=40
     Fx(k)=0.0; 
     Fy(k)=0.0;
   end
   if t(k)>=50
     Fx(k)=0.0;
     Fy(k)=-0.1;
   end

   %% Generacion del vector de errores del sistema
   hxe(k)=hxd(k)-hx(k);
   hye(k)=hyd(k)-hy(k);
   
   hxep(k)=hxdp(k)-hxp(k);
   hyep(k)=hydp(k)-hyp(k);
   %% Generacion de los vectortes generales de control
   hd=[hxd(k) hyd(k)]';
   
   hdp=[hxdp(k) hydp(k)]';
   
   hdpp=[hxdpp(k) hydpp(k)]';
   
   h=[hx(k) hy(k)]';
   
   hp=[hxp(k) hyp(k)]';
   
   %% Generacion del vector de Fuerzas
   F=[Fx(k);Fy(k)];
   
   %% Generacion de los estados internos del sistema
   q=[q1(k) q2(k)]';
   
   qp = [q1p(k) q2p(k)]';
   
   %% Controlador
   u(:,k)=force_control(qp,q,hdpp,hdp,hd,hp,h,Mm,Dm,Km,l,F);
   
   %% Dinamica del sistema
   [qpp] = dinamic(q,qp,u(:,k),F,l);
   
   %% Integracion de los valores
   q1p(k+1)=q1p(k)+To*qpp(1);
   q2p(k+1)=q2p(k)+To*qpp(2);
   
   %% integral de las acciones de control
   q1(k+1)=q1(k)+To*q1p(k+1);
   q2(k+1)=q2(k)+To*q2p(k+1);
   
   %% valores actualizados para estimacion posicon y velocidad
   q_real=[q1(k+1),q2(k+1)]';
   qp_real=[q1p(k+1),q2p(k+1)]';
   
   %% Cinematica directa para ver donde esta le istema
   [hx(k+1),hy(k+1)] = direct_kinematic(q_real,l);
   [hxp(k+1),hyp(k+1)] = direct_kinematic_dot(qp_real,q,l);
end
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t,hxe,'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t,hye,'Color',[46,188,89]/255,'linewidth',1); hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of position errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
subplot(2,1,2)
plot(t,hxep,'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t,hyep,'Color',[46,188,89]/255,'linewidth',1); hold on;
grid on;
legend({'$\tilde{h_{xp}}$','$\tilde{h_{yp}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of velocity error}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t,q1p(1:length(t)),'Color',[223,67,85]/255,'linewidth',1); hold on
plot(t,q2p(1:length(t)),'Color',[56,171,217]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{q}_{1p}$','$\dot{q}_{2p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
subplot(2,1,2)
plot(t,q1(1:length(t)),'Color',[223,67,85]/255,'linewidth',1); hold on
plot(t,q2(1:length(t)),'Color',[56,171,217]/255,'linewidth',1); hold on
grid on;
legend({'$q_{1}$','$q_{2}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,u(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t,u(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on;
grid on;
legend({'$T_1$','$T_2$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Torque Control}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t,hx(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t,hy(1,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t,Fy(1,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t,Fx(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on;
grid on;
legend({'$h_{x}$','$h_{y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution }$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);