%% controlller design


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
J=J1;   b=b1;     Ra=Ra1;   La=La1; Kt=Kt1;   Kb=Kb1;

J=J2;   b=b2;     Ra=Ra2;   La=La2; Kt=Kt2;   Kb=Kb2;


% simplified DC_motor parameters
K_mot=Kt/(b+Kt*Kb);
tau_mot=(Ra*J)/(b+Kt*Kb);

% sensor
K_sensor=12/(pi/4);

K_driver=1;


% Calculo Parametro controlador 
K_mot_prima=K_driver*K_mot*K_sensor;
K_p=1/(4*tau_mot*K_mot_prima);


s=tf('s');
close all
P_motor_simple=K_mot_prima/(s*(tau_mot*s+1))
P_motor=(K_driver*K_sensor*Kt)/(s*((J*s+b)*(La*s+Ra)+Kb*Kt));

%controlSystemDesigner(P_motor) 

for k=1:10
C = pid(K_p*(0.7+0.1*k));
sys_cl = feedback(C*P_motor,1);


  t = 0:dt:0.4;
  step(sys_cl,t);
  grid
  title('Step Response with Proportional Control')
  hold on
end

hold off

figure()
C = pid(K_p);
sys_cl = feedback(C*P_motor,1);

C2 = pidtune(P_motor,'P');
sys2 = feedback(C2*P_motor,1);

C3 = pidtune(P_motor,'PI');
sys3 = feedback(C3*P_motor,1);

C4 = pidtune(P_motor,'PID');
sys4 = feedback(C4*P_motor,1);

C5 = pid(1,0,tau_mot);
sys5 = feedback(C5*P_motor,1);

step(sys_cl,sys2,sys3,sys4,sys5,t)
legend("k_p","P","PI","PID","k_p=1 Kd=Tm")
%% alternative controller
SO=0.1;  % valor proporcional de sobrepico 
t_p=2*tau_mot; 
z= -log(SO)/(sqrt(pi^2+log(SO)));
w_n=pi/(t_p*sqrt(1-z^2));

 
%%  Calculo de parametros en velocidad
% t=omega.Time;
% 
% y= omega.Data;
% 
% plot(t,y)
% 
% y_max=max(y);
% K_mot_calc=y_max/A;
% 
% id=find(diff(y<y_max*0.63));
% 
% tau_mot_calc=t(id);
% 
% %% Calculo de parametros en posición
% t=theta.Time;
% 
% y= theta.Data;
% 
% plot(t,y)
% 
% p = polyfit(t,y,1);
% 
% m=p(1);
% b=(2);
% K_mot_calc=m/A;
% tau_mot_calc=b/m;
% 
