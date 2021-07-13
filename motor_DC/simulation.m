

mode=2;
% Parametros motor 1
t_end=20;

N1=5;

Kp1 = 3.16, Ki1 = 4.1, Kd1 = 0.0343

La1 = 2.93e-3;               %H
Ra1 = 0.89;               %Ohm

JM1=3.3e-4
J1 =J_barra(1)/N1^2;      %kg-m^2
RI1=J1/JM1;

b1=0.005;

Kt1 = 0.018;           %Nm / A
Kb1 = 0.018;           %V /(rad / s)


% Parametros motor 2


N2=15;

Kp2 = 0.146, Ki2 = 1.52, Kd2 = 0.00151;

% Kp2=6/N1;
% Ki2=0.5/N1;
% Kd2=2/N1;

La2 = 0.42e-3;           %H
Ra2 = 0.7;              %Ohm

JM2 =5.8e-6;
J2 =J_barra(2)/N2^2;        %kg-m^2
RI2=J2/JM2
b2=3.7e-6;

Kt2 = 0.0145;           %Nm / A
Kb2 = 0.0145;           %V /(rad / s)


sampling=500;
my_model=sim("motor_DC/servo_system_model.slx");

close all
time=my_model.theta_ref.Time;
w_ref=my_model.theta_ref.Data;

w2=my_model.theta_m.Data;

[var,S2_ref]=position(time,w_ref');

[S1,S2]=position(time,w2');

torque=my_model.T_m.Data;

%subaxis(4,6,1, 'Spacing', 0.03, 'Padding', 0, 'Margin', 0, 'SpacingVert', 0.03);
plot(time, w_ref(:,1),"--b",time, w2(:,1),"b")

plot(time, w_ref(:,2),"--r",time, w2(:,2),"r")

%%


filename = '../Simulation.gif';
capture=true;
h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

for k= 1:length(time)
  
subplot(1,2,1)
plot(S2_ref(1,1:k),S2_ref(2,1:k),"--g")
hold on

plot([0,S1(1,k)],[0,S1(2,k)], "b","LineWidth",4)
plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)],"r", "LineWidth",2)

plot(S2(1,1:k),S2(2,1:k),":r")

axis equal
ylim([-40,40]);

xlim([-40,40]);

title("trayectoria")
hold off
%title("Kp:" + Kp+"  Ki:"+ Ki+ "   Kd:" + Kd)


subplot(3,2,2)
style={"--","--","-","-"};
color={"blue","red","blue","red"};

var=plot(time,w_ref,time,w2);

% for i=1:4 
%   
% h(i).LineStyle=style{i};
% end
[var(:).LineStyle] = style{:};
[var(:).Color] = color{:};

title("\theta  [rad]")

hold off

subplot(3,2,4)
plot(time,w2-w_ref)
title("Error [rad]")

subplot(3,2,6)
plot(time,torque)
title("Torque [Nm]")

 drawnow

  if (capture)
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
