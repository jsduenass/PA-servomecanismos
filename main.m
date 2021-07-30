%% set up

clear, clc, close all
addpath('auxiliary_functions')
tic

run parameters.m

T=15;             % Periodo de giro 15 a 40 rad/s
Amp=1.3;          % factor de escala entre 1 y 1.3
phi=pi/2;            % desface

t_end=2.2*T;
wait=3;       % tiempo de espera entre plots
 sampling=100;
  

%% simulación 
close all
run simulation.m


%% Animación de simulación
%run animation_sim


%% perfiles de movimiento y calculo preliminar de torque
run motion_profiles.m


%% Euler vs Newtonian comparison
close all
run Euler_Lagrange.m


%% animación
run animation.m


%% diseño de controlador 

run control_design

toc






