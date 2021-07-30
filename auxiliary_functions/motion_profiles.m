% Calculo velocidad constante con vector de tiempo de espaciado no uniforme
% syms angle t
% [x,y]=trajectory(Amp,phi,K,angle);
% 
% velocity= sqrt(diff(x)^2+diff(y)^2);        % velocidad lineal
% 
% dt= matlabFunction(velocity/cruce_speed);  
% 
% angle=linspace(angle_ini,angle_ini+4*pi,300);   % vector angular
% 
% 
% tiempo=cumtrapz(angle,dt(angle));  % tiempo con espaciado no uniforme
% 
% % interpolación de angulo a tiempo con espaciado uniforme 
% t=linspace(tiempo(1),tiempo(end),length(tiempo));
% angle  = interp1(tiempo,angle,t);

% trayectoria a velocidad angular constante
%t=linspace(0,t(end),length(t));
t=linspace(0,20,500);
angle= 2*pi*t/T+angle_ini;
dt=t(2);


%% positioning from home to trajectory
 
 [x,y]=trajectory(Amp,phi,K,angle);

  n=ceil(min_d/(cruce_speed*dt))+1; 
  x_trans=linspace(x_home,x(1),n);    % Transtional trayectories
  y_trans=linspace(y_home,y(1),n);
  x=[x_trans, x ];
  y=[y_trans, y];
  t=[t,t(end)+(1:n)*dt];

  % perfil de movimiento angular
  theta_m=inverse_kinematic(x,y) ;
%%
load 'log.mat'


t=time';
dt=t(2);
theta_m=theta_traveled';

%
% perfil de velocidad angular
omega_m=gradient(theta_m,dt);

% perfil de aceleración angular
alpha_m=gradient(omega_m,dt);

%velocity=t;

% series de tiempo vector de posición 
S1=[]; S2=[];

% centros de masas
c1=[]; c2=[];

% Calculo iterativo posiciones y centros de masa 
for k= 1:length(t)
  
  theta1=theta_m(1,k);
  theta2=theta_m(2,k);
  P1=L1*R(theta1)*[1;0];          % pos extremo primer eslabon
  P21=L2*R(theta2+theta1)*[1;0];  % pos relativa primer a segundo eslabon
  P2=P21+P1;                      % pos extremo segundo eslabon  
  
  % centro de masa 2
  c2(:,end+1)= (m2*P21/2+mL*P21)./(m2+mL);                      
  % centro de masa 1
  c1(:,end+1)= (m2*(P21/2+P1) + mL*(P2+P1)+ mM2*P1 + m1*P1/2)./(m1+m2+mL+mM2);  
  
  S1(:,end+1)=P1;   % trayectoria eslabon 1
  S2(:,end+1)=P2;   % trayectoria eslabon 2
end

% Calculo de torque Dinamico 

Tf=b.*omega_m;       % Torque de fricción 
Tp=[0;0].*omega_m;       % Torque de proceso
Tg=9.81*[(m1+m2+mL+mM2)*c1(1,:);(m2+mL)*c2(1,:)]*10^-2;   % Torque gravitacional

Tm=alpha_m.*J_L + Tg +Tf + Tp;
Tm = Tm./N; 


x=S2(1,:);  y=S2(2,:);
velocity=vecnorm(gradient([x;y],dt),2,1);
figure()
plot(t,velocity)
title("velocidad [cm/s]")
xlabel("Tiempo [s]")
ylim([0,12])

pause(wait)

figure ()
subplot(3,1,1)
plot(t,theta_m)
title("\theta_m [rad]")
xlabel("Tiempo [s]")
grid on

subplot(3,1,2)
h=plot(t,omega_m); 
title("\omega_m [rad/s]")
xlabel("Tiempo [s]")
grid on

subplot(3,1,3)
plot(t,alpha_m)
title("\alpha_m [rad/s^2]")
xlabel("Tiempo [s]")
grid on


pause(wait)

% pause(2)
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

figure()
plot(t,Tm(1,:),t,Tm(2,:))
legend(["Torque Eslabon 1","Torque Eslabon 2"])
title("Torque de mecanismo [Nm]")
xlabel("Tiempo [s]")
grid on

limit_x=desp_x-r_max;
limit_y=desp_y;


pause(wait)
