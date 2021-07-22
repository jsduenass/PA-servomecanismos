clear 
close all
global desp_x desp_y r_base
% Parametros de trayectoria trevol estilizado
T=10;             % Periodo de giro
r_base=7.5;       % radio base [cm]
Amp=1.3;          % factor de escala entre 1 y 1.3
phi=0;         % desface
K=0.4;            % factor de estilizado entre 0 y 1  
cruce_speed=4;
r_max=Amp*r_base*(1+K);
desp_x=22;    desp_y=10;
t=linspace(0,2*pi,300);
% calculo angulo inicial de trayectoria como aquel más cercano a la posición home 
[x,y]=trayectory(Amp,phi,K,t);


[x_sen,y_sen]=trayectory_mod(Amp,phi,K,t);
%y_sen=-K*r_base*Amp*cos(4*t-phi);
x_r=Amp*r_base*cos(t)+desp_x;
y_r=Amp*r_base*sin(t)+desp_y;
r=-Amp*r_base*(K*cos(4*t-phi));

filename = './media/ref.avi';
capture=true;

h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

%close(v)
v = VideoWriter(filename,'Motion JPEG AVI')
 
open(v)

for k= 1:length(t)
  
plot(x(1:k),y(1:k),"g")
hold on

plot(x_r(1:k),y_r(1:k))
plot(35+4*t(1:k),desp_y+r(1:k),'Color',[0.9290,0.6940, 0.1250])
plot([34,60],[desp_y,desp_y],'k')
plot([35,35],[5, 15],'k')
%plot(x_sen(1:k),y_sen(1:k))
plot([x_r(k),x(k)],[y_r(k),y(k)],'Color',[0.9290,0.6940, 0.1250])
axis equal
ylim([-10,40]);

xlim([-10,65]);


title("trayectoria")
hold off

%pause(0.1)
  drawnow
  if (capture )
       % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    writeVideo(v,im)
  end

end

close(v)
function [x,y]=trayectory_mod(Amp,phi,K,angle)
global  r_base desp_x desp_y 


r=4+Amp*r_base*(K*cos(4*angle-phi));

% Desplazamiento de trayectoria
[x,y]=pol2cart(angle,r);  

x=x+desp_x;
y=y+desp_y;

end

