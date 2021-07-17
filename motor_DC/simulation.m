
run '../main.m'
%%
% Parametros motor 1
  N1=5;
  N1=1;
  
  Kp1 = 2.16, Ki1 = 0.3, Kd1 = 0.3
  Kp1 = 3, Ki1 = 0;     Kd1 = 0.1;
  
  La1 = 2.93e-3;               %H
  Ra1 = 0.89;               %Ohm


  J1 =J_barra(1)/N1^2;      %kg-m^2
  b1=0.005;

  RI1=J1/J_m1;

  Kt1 = 0.018;           %Nm / A
  Kb1 = 0.018;           %V /(rad / s)


% Parametros motor 2

  N2=15;
  N2=1;

  Kp2 = 2.46, Ki2 = 0.5, Kd2 = 0.00;
  Kp2 = 3;  Ki2=0.1;  Kd2=0.5;
  
  La2 = 0.42e-3;           %H
  Ra2 = 0.7;              %Ohm

  J2 =J_barra(2)/N2^2;        %kg-m^2
  b2=3.7e-6;

  RI2=J2/J_m2

  Kt2 = 0.0145;           %Nm / A
  Kb2 = 0.0145;           %V /(rad / s)


% Parametros simulacion
  t_end=10;
  dt=1e-3;
  sampling=50;

  % sensor
  K_sensor=12/(pi/4);

  % condicion inicial
  theta1_ini=theta_m(1,1);
  theta2_ini=theta_m(2,1);
  %theta1_ini=-pi/2;
  %theta2_ini=0;


  mode=2;       % Modo  referencia de simulación 1: paso 2: trayectoria 3:zero


load 'log.mat'
  
%my_model=sim("motor_DC/servo_system_model.slx");

my_model=sim("../Motor_SimScape.slx");

close all
time=my_model.theta_ref.Time;
theta_ref=my_model.theta_ref.Data;

theta_traveled=my_model.theta_m.Data;

[S1_ref,S2_ref]=position(time,theta_ref');

[S1,S2]=position(time,theta_traveled');

torque=my_model.T_m.Data;

trayectory1=[S1_ref;ones(size(time'))]'+0.01*rand(length(time),3);

trayectory2=[S2_ref;4.5*ones(size(time'))]'+0.01*rand(length(time),3);

save('log.mat','trayectory1','trayectory2')

%subaxis(4,6,1, 'Spacing', 0.03, 'Padding', 0, 'Margin', 0, 'SpacingVert', 0.03);
plot(time, theta_ref(:,1),":b",time, theta_traveled(:,1),"b")

plot(time, theta_ref(:,2),":r",time, theta_traveled(:,2),"r")

%% Animación


filename = '../Simulation.gif';
capture=false;
h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

for k= 1:5:length(time)
  
subplot(1,2,1)
plot(S2_ref(1,1:k),S2_ref(2,1:k),"--g")
hold on

plot([0,S1(1,k)],[0,S1(2,k)], "b","LineWidth",4)
plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)],"r", "LineWidth",2)

plot(S2(1,1:k),S2(2,1:k),":r")

xline(limit_x,"--")
yline(limit_y,"--")

axis equal
ylim([-40,40]);

xlim([-40,40]);

title("trayectoria")
hold off
%title("Kp:" + Kp+"  Ki:"+ Ki+ "   Kd:" + Kd)


subplot(3,2,2)
style={"--","--","-","-"};
color={"blue","red","blue","red"};

var=plot(time,theta_ref,time,theta_traveled);

[var(:).LineStyle] = style{:};
[var(:).Color] = color{:};

ylabel("\theta  [rad]")
xlabel("tiempo[s]")
hold off

subplot(3,2,4)
plot(time,theta_traveled-theta_ref)
yline(0)
ylabel("Error [rad]")
xlabel("tiempo[s]")

subplot(3,2,6)
plot(time,torque)
ylabel("Torque [Nm]")
xlabel("tiempo[s]")
  drawnow
  if (capture & mod(k,20)==1)
   
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',16*n*dt); 
    end 
  end

end


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
