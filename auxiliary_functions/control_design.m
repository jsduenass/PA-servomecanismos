%% Caracterización del sistema
t_end=20;
t_end=0.4
theta1_ini=-pi/2;

theta1_ini=0;
theta2_ini=0;
dt = 1e-4;
sampling=1;
A=pi/64;
motor_id=2;     %select between motor 1 and motor 2;

my_model=sim("./sim_models/servo_open_system.slx");



time=my_model.theta_m.Time;

theta_traveled=my_model.theta_m.Data;

pos=theta_traveled(:,motor_id);

vel = gradient(pos,time(2));
 
figure()
  subplot(2,1,1)
  plot(time,pos)
  grid on 
  title("\theta [rad/s]")
  xlabel("t [s]")
  
  subplot(2,1,2)
  plot(time,vel)
  grid on 
  title("\omega [rad/s]")
  xlabel("t [s]")
  
pause(wait)
%%  Calculo de parametros en velocidad
 t=time;
 y=vel;

y_max=max(y);
K_mot_calc=y_max/A

id=find(diff(y<y_max*0.63));

tau_mot_calc=t(id)


figure()
plot(t,vel)
title("\omega [rad/s]")
yline(0)
xline(tau_mot_calc,"--")
yline(y_max,"--")

xlabel("t [s]")

dim = [.5 .5 .4 .2];
str = "K_{mot} : " +K_mot_calc+newline+newline+" tau_{mot} : " + tau_mot_calc;
annotation('textbox',dim,'String',str,'EdgeColor','none')

pause(wait)

% Resultados: 
% M1 theta=-pi/2 
  % K_mot_calc =  6.4985
  % tau_mot_calc = 0.0156

% M1 theta=0
  % K_mot_calc =  4.5930
  % tau_mot_calc = 0.0176
  
% M2 theta=-pi/2
  % K_mot_calc =  57.6482
  % tau_mot_calc = 0.02465

%% Calculo de parametros en posición
t=time;

y=pos - pos(1);

% Resultados: 
p = polyfit(t,y,1);

m=p(1);
b=p(2);
K_mot_calc=m/A
tau_mot_calc=-b/m

figure()
plot(t,y, t, m*t+b,"--")
grid on 
title("\theta [rad/s]")
xlabel("t [s]")
yline(0)
xline(tau_mot_calc,"--")

dim = [.22 .5 .4 .2];
str = "K_{mot} : " +K_mot_calc+newline+newline+" tau_{mot} : " + tau_mot_calc;
annotation('textbox',dim,'String',str,'EdgeColor','none')
pause(wait)

% M1 theta=-pi/2
  % K_mot_calc = 6.2708
  % tau_mot_calc = 0.0114


  
% M1 theta=0
  % K_mot_calc =  4.5930
  % tau_mot_calc = x

% M2 theta=-pi/2
  % K_mot_calc =  51.2213
  % tau_mot_calc = 0.0203

  
%% controller design

K_mot=5.4;
tau_mot=0.023;
s=tf('s');
P_motor=K_mot/(s*(tau_mot*s+1))

% criterios de diseño
SO=0.01;     % valor proporcional de sobrepico 
t_p=0.1;     % tiempo pico
  
% respuesta criticamente amortiguada
  K_p= 1/(4*tau_mot*K_mot);

  C1 = pid(K_p,0,0);
  sys1 = feedback(C1*P_motor,1);

% por sobrepico /zeta
  z= -log(SO)/(sqrt(pi^2+log(SO)^2));

  Kp2= 1/(4*z^2*tau_mot*K_mot);

  C2 = pid(Kp2,0,0);
  sys2 = feedback(C2*P_motor,1);


% por tiempo pico 
  z= -log(SO)/(sqrt(pi^2+log(SO)^2));
  w_n=pi/(t_p*sqrt(1-z^2));

  Kp=w_n^2*(tau_mot/K_mot)

  Kd=(2*z*w_n*tau_mot-1)/K_mot


  C3 = pid(Kp,0,Kd);
  sys3 = feedback(C3*P_motor,1);


  C4 = pid(Kp,0,0);
  sys4 = feedback(C4*P_motor,1);
  
figure()
step(sys1,sys2,sys3,sys4);
xline(t_p,"--","t_{pico}")
yline(1+SO,"--r","Max sobre pico ")
str_temp="t_{pico}: P="+ Kp +" D="+Kd;
str=["crit amort:  P=" + K_p,"\zeta:      P="+Kp2 ,str_temp  ,"t_{pico}: P="+ Kp];
legend(str, "Location","southeast")
pause(wait)

figure()
bode(P_motor,sys2,sys3)
legend(["modelo motor","controlador P","controlador PD"])


%%
% parametros motor DC generico
J = 1.62e-6 ;         

La = 0.58e-3;         %H
Ra = 1.17;            %Ohm
b = 1.34e-6;          %Nm /(rad / s)
Kt = 0.011;           %Nm / A
Kb = 0.011;           %V /(rad / s)
tau_e = 0.50e-3;      %s
tau_m = 16e-3;        %s
tau_t = 660;          %s

% Adaptar parametros a motor 1 o motor 2
%J=Jm1;   b=b1;     Ra=Ra1;   La=La1; Kt=Kt1;   Kb=Kb1;

J=Jm2;   b=b2;     Ra=Ra2;   La=La2; Kt=Kt2;   Kb=Kb2;


% simplified DC_motor parameters
% K_mot=Kt/(b+Kt*Kb);
% tau_mot=(Ra*J)/(b+Kt*Kb);

K_driver=1;
% Calculo Parametro controlador 
K_mot_prima=K_driver*K_mot*K_sensor;
K_p= 1/(4*tau_mot*K_mot_prima);


s=tf('s');

P_motor=(K_driver*K_sensor*Kt)/(s*((J*s+b)*(La*s+Ra)+Kb*Kt));
%P_motor=K_mot_prima/(s*(tau_mot*s+1))
%bode(P_motor)

%controlSystemDesigner(P_motor) 
% 
% for k=1:10
%   C = pid(K_p*(0.7+0.1*k));
%   sys_cl = feedback(C*P_motor,1);
%   t = 0:dt:0.4;
%   
% %   step(sys_cl,t);
% %   grid
% %   title('Step Response with Proportional Control')
% %   hold on
% end
% 
% hold off
% 
% figure()
% C = pid(K_p);
% sys_cl = feedback(C*P_motor,1);
%%
s=tf('s');
P_motor=K_mot/(s*(tau_mot*s+1));

C2 = pidtune(P_motor,'P');
sys2 = feedback(C2*P_motor,1);

C3 = pidtune(P_motor,'PI');
sys3 = feedback(C3*P_motor,1);

C4 = pidtune(P_motor,'PID');
sys4 = feedback(C4*P_motor,1);

C5 = pid(1,0,tau_mot);
sys5 = feedback(C5*P_motor,1);

% step(sys2,sys3,sys4,sys5,5)
% legend("P","PI","PID","k_p=1 Kd=Tm")


 

