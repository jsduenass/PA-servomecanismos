% sim parameters
t_end=2;
dt=1e-6;

dt=1e-4;

DC_model_type=1;      % 1-> complete 2-> simple
 

theta1_ini=theta_m(1,1)
theta2_ini=theta_m(2,1)

% DC_motor parameters
J = 1.62e-6 ;         

La = 0.58e-3;         %H
Ra = 1.17;            %Ohm
b = 1.34e-6;          %Nm /(rad / s)
Kt = 0.011;           %Nm / A
Kb = 0.011;           %V /(rad / s)
tau_e = 0.50e-3;      %s
tau_m = 16e-3;        %s
tau_t = 660;          %s

% Sistema de transmición de potencia
N=5;                  %gear ration

% simplified DC_motor parameters
K_mot=Kt/(b+Kt*Kb);
tau_mot=(Ra*J)/(b+Kt*Kb);

% sensor
K_sensor=12/(pi/4);

K_driver=1;


% Calculo Parametro controlador 
K_mot_prima=K_driver*K_mot*K_sensor;
K_p=1/(4*tau_mot*K_mot_prima);



