%% set up

clear, clc, close all
addpath('auxiliary_functions')
tic

run parameters.m

T=15;             % Periodo de giro 15 a 40 rad/s
Amp=1.2;          % factor de escala entre 1 y 1.3
phi=60*pi/180;    % desface

t_end=2.2*T;

wait=3;       % tiempo de espera entre plots
sampling=100;

mode=2;         % Modo  referencia de simulación 1: paso 2: trayectoria 3:zero

% Respuesta al paso sistema cerrado
%   T_s=1e-4;
%   t_end=0.4;
%   
%   mode=1;       
% 

%% simulación 
close all
run simulation.m


%% Animación de simulación
run animation_sim


%% perfiles de movimiento y calculo preliminar de torque
run motion_profiles.m


%% Euler vs Newtonian comparison
close all
run Euler_Lagrange.m


%% animación
run animation.m


%% diseño de controlador 
close all
run control_design

toc






