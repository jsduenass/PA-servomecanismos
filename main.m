clear, clc, close all


% Parametros de mecanismo
global L1 L2 desp_x desp_y  r_base
L1=22;    L2=20;
desp_x=24;
desp_y=14;

x_home=8; y_home=-8;

m1=0.4;   m2=0.3;   mL=0.2;


% Parametros de trayectoria trevol estilizado

r_base=8.5;       % radio base [cm]
K=1.1;            % factor de escala entre 1 y 1.2
phi=pi/4;         % desface
A=0.4;            % factor de estilizado entre 0 y 1  
cruce_speed=4;

r_max=K*r_base*(1+A);

% calculo angulo inicial de trayectoria como aquel más cercano a la posición home 
[x,y]=trayectory(K,phi,A,linspace(0,2*pi,100));

d=sqrt((x-x_home).^2+(y-y_home).^2);
[min_d,id]=min(d);

initial_angle=2*pi*id/100;

% Calculo velocidad constante con vector de tiempo de espaciado no uniforme
syms angle t
[x,y]=trayectory(K,phi,A,angle);

velocity= sqrt(diff(x)^2+diff(y)^2);        % velocidad lineal

dt= matlabFunction(velocity/cruce_speed);  

angle=linspace(initial_angle,initial_angle+4*pi,300);   % vector angular

tiempo=cumtrapz(angle,dt(angle));  % tiempo con espaciado no uniforme

% interpolación de angulo a tiempo con espaciado uniforme 
t=linspace(tiempo(1),tiempo(end),length(tiempo));
angle  = interp1(tiempo,angle,t);

dt=t(2);
%% positioning from home to trayectory

[x,y]=trayectory(K,phi,A,angle);

N=ceil(min_d/(cruce_speed*dt))+1; 
x_trans=linspace(x_home,x(1),N);    % Transtional trayectories
y_trans=linspace(y_home,y(1),N);
x=[x_trans, x, flip(x_trans)];
y=[y_trans, y, flip(y_trans)];
t=[t,t(end)+(1:2*N)*dt];

% perfil de movimiento angular
theta_m=inverse_kinematic(x,y) ;

% perfil de velocidad angular
omega_m=gradient(theta_m,dt);

% perfil de aceleración angular
alpha_m=gradient(omega_m,dt);

velocity=vecnorm(gradient([x;y],dt),2,1);

% series de tiempo vector de posición 
S1=[]; S2=[];

% centros de masas
c1=[]; c2=[];

for k= 1:length(t)
  
  theta1=theta_m(1,k);
  theta2=theta_m(2,k);
  P1=L1*R(theta1)*[1;0];
  P21=L2*R(theta2+theta1)*[1;0];
  P2=P21+P1;
  
  c2(:,end+1)= (m2*P21/2+mL*P21)./(m2+mL);                      % centro de masa 1
  c1(:,end+1)= (m2*(P21/2+P1) + mL*P2 + m1*P1/2)./(m1+m2+mL);   % centro de masa 2
  
  S1(:,end+1)=P1;   % trayectoria eslabon 1
  S2(:,end+1)=P2;   % trayectoria eslabon 2
end

% Calculo de torque Dinamico 

Tf=0.1*omega_m;       % Torque de fricción 
Tp=[2,3]*omega_m;       % Torque de proceso
Tg=-9.81*[(m1+m2+mL)*c1(1,:);(m2+mL)*c2(1,:)]*10^-2;   % Torque gravitacional
J_L=[3;3];      % Momento de inercia

Tm=alpha_m.*J_L - Tg -Tf - Tp;
%Tm=Tg ;

%close all

plot(t,velocity)
title("velocidad [cm/s]")
ylim([0,12])

figure ()
subplot(3,1,1)
plot(t,theta_m)
title("\theta_m [rad]")
grid on

subplot(3,1,2)
h=plot(t,omega_m); 
title("\omega_m [rad/s]")
grid on

subplot(3,1,3)
plot(t,alpha_m)
title("\alpha_m [rad/s^2]")
grid on

%pause(2)
% 
% figure()
% 
% plot(S2(1,:),S2(2,:))
% axis on
% hold on
% plot(x,y, "--r","LineWidth",1.4)
% xlim([-20,20])
% ylim([-20,20])
% title("trayectoria")
% axis equal
% hold off

%pause(2)
figure()
plot(t,Tm)
title("Torque de motor [Nm]")
grid on

limit_x=desp_x-r_max;
limit_y=desp_y;

%%
pause(1)

%h=figure()
%h=figure('units','normalized','outerposition',[0 0 1 1])   %full screen

h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

axis tight manual % this ensures that getframe() returns a consistent size
filename = 'Simulation.gif';
capture=true;

for k= 1:length(t)  
  subplot(2,2,1)
  %arm
  plot([0,S1(1,k)],[0,S1(2,k)], "LineWidth",4)
  hold on
  plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)], "LineWidth",2)
  plot(c1(1,k),c1(2,k),"ob")
  plot(c2(1,k)+S1(1,k),c2(2,k)+S1(2,k),"or")
  
  % work area
  %rectangle('Position',workArea,"LineStyle","--")  
  xline(limit_x,"--")
  yline(limit_y,"--")
  %target trajectory
  plot(x,y)
  
  title("trayectoria")
  grid on
  axis equal
  xlim([-14,max(x)+2])
  ylim([-20,max(y)+2])
  hold off
  
  
  subplot(3,2,5)
  plot(t(1:k),Tm(:,1:k))
  title("Torque de motor [Nm]")
  grid on
  xlim([0,t(end)])
  ylim([min(Tm,[],"all"),max(Tm,[],"all")])
  
  subplot(3,2,2)
  plot(t(1:k),theta_m(:,1:k))
  title("\theta_m [rad]")
  xlim([0,t(end)])
  ylim([min(theta_m,[],"all"),max(theta_m,[],"all")])
  grid on

  plot(t(1:k),velocity(1:k))
  title("velocidad [cm/s]")
  ylim([0,12])
  xlim([0,t(end)])
  
  
  subplot(3,2,4)
  plot(t(1:k),omega_m(:,1:k)); 
  title("\omega_m [rad/s]")
  xlim([0,t(end)])
  ylim([min(omega_m,[],"all"),max(omega_m,[],"all")])
  grid on

  subplot(3,2,6)
  plot(t(1:k),alpha_m(:,1:k))
  title("\alpha_m [rad/s^2]")
  xlim([0,t(end)])
  ylim([min(alpha_m,[],"all"),max(alpha_m,[],"all")])
  
  grid on

  drawnow
  n=5
  if (capture & mod(k,n)==1)
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',n*dt); 
    end 
  end

end



%%

 

function rot=R(theta_m)
  rot=[cos(theta_m),-sin(theta_m);sin(theta_m),cos(theta_m)];
end






