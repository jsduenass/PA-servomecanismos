
function [x,y]=trajectory(Amp,phi,K,angle)
global  r_base desp_x desp_y 


r=Amp*r_base*(1-K*cos(4*angle-phi));

% Desplazamiento de trayectoria
[x,y]=pol2cart(angle,r);  

x=x+desp_x;
y=y+desp_y;

end

