
run 'main.m'
%%
close all

motors= readtable('motors.xlsx');

id_M1=7;

id_M2=5;

param=table2cell( motors(id_M1,:));
[Ra1,La1,Jm1,b1,mM1,Kb1,Kt1]=deal(param{:});

param=table2cell( motors(id_M2,:));
[Ra2,La2,Jm2,b2,mM2,Kb2,Kt2]=deal(param{:});

% Parametros motor 1
  N1=10;
  %N1=1;
  
  Kp1 = 2.16, Ki1 = 0.3, Kd1 = 0.3
  Kp1 = 2.88 , Ki1 = 0;     Kd1 = 0;
  %Kp1 = 1 , Ki1 = 0;     Kd1 = 0.1;
  
  J1 =(J_barra(1)+J_barra(2))/N1^2;      %kg-m^2
  
  RI1=J1/Jm1;

  

% Parametros motor 2

  N2=16;
  %N2=1;
  
  Kp2 = 2.46, Ki2 = 0.5, Kd2 = 0.50;
  Kp2 = 2.81;  Ki2=0;  Kd2=0;
  %Kp2 = 1;  Ki2=0;  Kd2=0.03;
  
  
  J2 =J_barra(2)/N2^2;        %kg-m^2
  
  RI2=J2/Jm2


% Parametros simulacion
  t_end=20.5;
  dt=1e-3;
  sampling=50;

  % sensor
  K_sensor=5/pi;
  K_sensor=15;
  
  % condicion inicial
  theta1_ini=theta_m(1,1);
  theta2_ini=theta_m(2,1);

  mode=2;       % Modo  referencia de simulación 1: paso 2: trayectoria 3:zero


load 'log.mat'
  
my_model=sim("./sim_models/servo_system_complete.slx");

%%
time=my_model.theta_ref.Time;
theta_ref=my_model.theta_ref.Data;

theta_traveled=my_model.theta_m.Data;

[S1_ref,S2_ref]=position(time,theta_ref');

[S1,S2]=position(time,theta_traveled');

torque=my_model.T_m.Data;

T_ref=my_model.T_ref.Data;

ref1=[S1_ref;ones(size(time'))]'+0.01*rand(length(time),3);

ref2=[S2_ref;-0.6*ones(size(time'))]'+0.01*rand(length(time),3);

trayectory=[S2;-0.8*ones(size(time'))]'+0.01*rand(length(time),3);

save('log.mat','trayectory','ref1','ref2')

%subaxis(4,6,1, 'Spacing', 0.03, 'Padding', 0, 'Margin', 0, 'SpacingVert', 0.03);
plot(time, theta_ref(:,1),":b",time, theta_traveled(:,1),"b")

plot(time, theta_ref(:,2),":r",time, theta_traveled(:,2),"r")




figure()
subplot(2,1,1)
  plot(t, Tm(1,:),time,T_ref(:,1))
  legend(["Newtonian","T ref"])
  title("Torque Motor 1")
  subplot(2,1,2)
  plot(t, Tm(2,:),time,T_ref(:,2))
  legend(["Newtonian","T ref"])
  title("Torque Motor 2")

  figure()
plot(time,T_ref);

style={"--","--","-","-"};
color={"blue","red","blue","red"};

g=plot(time,T_ref,time,torque);

[g(:).LineStyle] = style{:};
[g(:).Color] = color{:};
title("Torque [N/m]")
legend(["inverse Dynamics1","inverse Dynamics2","T sim1","T sim1"])

%% Animación
run animation_sim
%%
function [S1,S2]=position(t,theta_m)
  global L1 L2
  S1=[];    S2=[];
  for k= 1:length(t)

    theta1=theta_m(1,k);
    theta2=theta_m(2,k);
    P1=L1*R(theta1)*[1;0];
    P21=L2*R(theta2+theta1)*[1;0];
    P2=P21+P1;

    S1(:,end+1)=P1;   % trayectoria eslabon 1
    S2(:,end+1)=P2;   % trayectoria eslabon 2
  end
  
  function rot=R(theta_m)
    rot=[cos(theta_m),-sin(theta_m);sin(theta_m),cos(theta_m)];
  end

end
