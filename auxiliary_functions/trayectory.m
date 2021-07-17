
function [x,y]=trayectory(K,phi,A,angle)
global  r_base desp_x desp_y 


r=K*r_base*(1-A*cos(4*angle-phi));

% Desplazamiento de trayectoria
[x,y]=pol2cart(angle,r);  

x=x+desp_x;
y=y+desp_y;

end

