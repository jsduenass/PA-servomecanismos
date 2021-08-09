
% Parametros motor 1
  Kp1 = 2.88 , Ki1 = 0;     Kd1 = 0;
  
  %Kp1 = 13 , Ki1 = 0;     Kd1 = 0.2;
  
  J1 =(J_barra(1)+J_barra(2))/N1^2;      %kg-m^2
  
  RI1=J1/Jm1;

  
% Parametros motor 2
  Kp2 = 2.81;  Ki2=0;  Kd2=0;
    
  J2 =J_barra(2)/N2^2;        %kg-m^2
  
  RI2=J2/Jm2

  
  % condicion inicial
  theta_ini=inverse_kinematic(x_home,y_home) ;
  theta1_ini=theta_ini(1);
  theta2_ini=theta_ini(2);


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

V_control=my_model.V_control.Data;

ref1=[S1_ref;ones(size(time'))]'+0.01*rand(length(time),3);

ref2=[S2_ref;-0.6*ones(size(time'))]'+0.01*rand(length(time),3);

traject=[S2;-0.8*ones(size(time'))]'+0.01*rand(length(time),3);

save('log.mat','traject','ref1','ref2','time','theta_traveled')

%subaxis(4,6,1, 'Spacing', 0.03, 'Padding', 0, 'Margin', 0, 'SpacingVert', 0.03);
%plot(time, theta_ref(:,1),":b",time, theta_traveled(:,1),"b")

%plot(time, theta_ref(:,2),":r",time, theta_traveled(:,2),"r")



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
