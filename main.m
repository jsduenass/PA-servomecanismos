clear, clc, close all

%
% 
% run motor_DC/parameters.m
% 
% my_model=sim("motor_DC/servo_system_model.slx");
% 
% w1=my_model.theta_m.Data;



% Parametros 
L1=22;    L2=17;
m1=0.4;   m2=0.3;   mL=0.3;

% Definición de trayectoria
angle=linspace(0,4*pi,300);   % 2 revolciones

w=0.65;   % velocidad de trayectoria

t=angle/w;
dt=t(2);

% parametros de trayectoria trevol estilizado
K=1;            % factor de escala entre 1 y 1.2
phi=pi*0;       % desface
A=0.4;          % factor de estilizado entre 0 y 1  
r_base=8;       % radio base [cm]

r=K*r_base*(1-A*cos(4*angle-phi));



% Desplazamiento de trayectoria
[x,y]=pol2cart(angle,r);  

desp_x=12;
desp_y=20;

% dx=27;
% dy=9;

x=x+desp_y;
y=y+desp_y;


[angle,r]=cart2pol(x,y);  

velocidad=vecnorm(gradient([x;y],dt),2,1);


%
L=sqrt(x.^2+y.^2);

% perfil de movimiento angular
theta=[angle-acos((L.^2+L1^2-L2^2)./(2*L*L1));
             acos((L.^2-L1^2-L2^2)/(2*L1*L2))];
% perfil de velocidad angular
omega=gradient(theta,dt);

% perfil de aceleración angular
alpha=gradient(omega,dt);

% series de tiempo vector de posición 
S1=[]; S2=[];

% centros de masas
c1=[]; c2=[];

for k= 1:length(t)
  
  theta1=theta(1,k);
  theta2=theta(2,k);
  P1=L1*R(theta1)*[1;0];
  P21=L2*R(theta2+theta1)*[1;0];
  P2=P21+P1;
  
  c2(:,end+1)= (m2*P21/2+mL*P21)./(m2+mL);                      % centro de masa 1
  c1(:,end+1)= (m2*(P21/2+P1) + mL*P2 + m1*P1/2)./(m1+m2+mL);   % centro de masa 2
  
  S1(:,end+1)=P1;   % trayectoria eslabon 1
  S2(:,end+1)=P2;   % trayectoria eslabon 2
end

% Calculo de torque Dinamico 

Tf=0.1*omega;       % Torque de fricción 
Tp=[2,3]*omega;       % Torque de proceso
Tg=-9.81*[(m1+m2+mL)*c1(1,:);(m2+mL)*c2(1,:)]*10^-2;   % Torque gravitacional
J_L=[3;3];      % Momento de inercia

Tm=alpha.*J_L - Tg -Tf - Tp;
%Tm=Tg ;

%close all

plot(t,velocidad)
title("velocidad [cm/s]")

figure ()
subplot(3,1,1)
plot(t,theta)
title("\theta [rad]")
grid on

subplot(3,1,2)
h=plot(t,omega); 
title("\omega [rad/s]")
grid on

subplot(3,1,3)
plot(t,alpha)
title("\alpha [rad/s^2]")
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


workArea=[min([S1,S2],[],2);max([S1,S2],[],2)-min([S1,S2],[],2)];
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
  
  %target trajectory
  plot(x,y)
  
  title("trayectoria")
  grid on
  axis equal
  xlim([-10,18+dx])
  ylim([-12,18+dy])
  hold off
  
  subplot(3,2,5)
  plot(t(1:k),Tm(:,1:k))
  title("Torque de motor [Nm]")
  grid on
  xlim([0,t(end)])
  ylim([min(Tm,[],"all"),max(Tm,[],"all")])
  
  subplot(3,2,2)
  plot(t(1:k),theta(:,1:k))
  title("\theta [rad]")
  xlim([0,t(end)])
  ylim([min(theta,[],"all"),max(theta,[],"all")])
  
  grid on

  subplot(3,2,4)
  plot(t(1:k),omega(:,1:k)); 
  title("\omega [rad/s]")
  xlim([0,t(end)])
  ylim([min(omega,[],"all"),max(omega,[],"all")])
  grid on

  subplot(3,2,6)
  plot(t(1:k),alpha(:,1:k))
  title("\alpha [rad/s^2]")
  xlim([0,t(end)])
  ylim([min(alpha,[],"all"),max(alpha,[],"all")])
  
  grid on

  drawnow
  if (capture & mod(k,5)==1)
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',5*dt); 
    end 
  end

end



%%

 

function rot=R(theta)
  rot=[cos(theta),-sin(theta);sin(theta),cos(theta)];
end






