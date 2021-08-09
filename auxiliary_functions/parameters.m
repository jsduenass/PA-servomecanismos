
% Parametros simulacion
  t_end=20;
  T_s=1e-3;       % tiempo de muestreo
  
  wait=0.3;       % tiempo de espera entre plots
  
% Parametros de mecanismo
global L1 L2 desp_x desp_y  r_base
% Distancia [cm]
L1=22;    L2=20;
desp_x=22;    desp_y=10;
x_home=4;     y_home=-10;


displacement=[desp_x;desp_y];
home=[x_home;y_home];

% masa [kg]
m1=0.455*L1/100;   % masa barra 1
m2=0.455*L2/100;   % masa barra 2
mL=0.02;           % masa carga

% Parametros de motor
motors= readtable('motors.xlsx');


% sensor
K_sensor=15;

% motors id from table 
id_M1=7;      
id_M2=5;        

% extracción de parametros
param=table2cell( motors(id_M1,:));
[Ra1,La1,Jm1,b1,mM1,Kb1,Kt1]=deal(param{:});

param=table2cell( motors(id_M2,:));
[Ra2,La2,Jm2,b2,mM2,Kb2,Kt2]=deal(param{:});


b=[b1;b2];       % coeficiente de fricción de sistema [Nm/s] 

N1=10;

N2=16;

N= [N1;N2];             % razon sistema de transmisión
N=[1;1];

J_barra=1/3*[(m1*(L1/100)^2);(m2*(L2/100)^2)];                        % Momento de inercia

J_L=J_barra+[Jm1;Jm2];


% Parametros de trayectoria trevol estilizado
T=15;             % Periodo de giro
r_base=7.5;       % radio base [cm]
Amp=1.3;          % factor de escala entre 1 y 1.3
phi=0;            % desface
K=0.4;            % factor de estilizado entre 0 y 1  
cruce_speed=4;
r_max=Amp*r_base*(1+K);

limit_x=desp_x-r_max;
limit_y=desp_y;

% calculo angulo inicial de trayectoria como aquel más cercano a la posición home 
[x,y]=trajectory(Amp,phi,K,linspace(0,2*pi,100));


d=sqrt((x-x_home).^2+(y-y_home).^2);
[min_d,id]=min(d);

angle_ini=2*pi*id/100;


r_ini=Amp*r_base*(1-K*cos(4*angle_ini+phi));

[x,y]=pol2cart(angle_ini,r_ini);  

x=x+desp_x;   y=y+desp_y;

d=sqrt((x-x_home).^2+(y-y_home).^2);
vel=(r_base*2*pi/T);

t_start=d/vel;


angle_ini= angle_ini + pi*t_start/T;

