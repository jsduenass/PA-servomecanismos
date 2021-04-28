clear, clc, close all

%trayectoria
angle=linspace(0,4*pi,200);
t=angle;
dt=t(2)
L1=6;    L2=8;
r=8+2*cos(4*angle+pi);
[x,y]=pol2cart(angle,r);
x=x/2+6;
y=y/2+6;
[angle,r]=cart2pol(x,y);

% 
% x=(4+0.2*t).*cos(t);      y=(6-0.2*t).*sin(t);
%atan2(y,x)
L=sqrt(x.^2+y.^2);
theta=[angle-acos((L.^2+L1^2-L2^2)./(2*L*L1));
             acos((L.^2-L1^2-L2^2)/(2*L1*L2))];
 
omega=gradient(theta);
alpha=gradient(omega);

S1=[]; S2=[];

% centro de masa
m1=8;   m2=4;   mL=1;
c1=[]; c2=[];
for k= 1:length(t)
  
  theta1=theta(1,k);
  theta2=theta(2,k);
  P1=L1*R(theta1)*[1;0];
  P21=L2*R(theta2+theta1)*[1;0];
  P2=P21+P1;
  
  c2(:,end+1)= (m2*P21/2+mL*P21)./(m2+mL);
  c1(:,end+1)= (m2*(P21/2+P1) + mL*P2 + m1*P1/2)./(m1+m2+mL);
  
  S1(:,end+1)=P1;
  S2(:,end+1)=P2;
end

Tf=[1;2];
Tp=[2;3];
Tg=-9.81.*[(m1+m2+mL)*c1(1,:);(m2+mL)*c2(1,:)];
J_L=[7;3];

Tm=alpha.*J_L-Tg-Tf-Tp;

figure ()
subplot(3,1,1)
plot(t,theta)
title("\theta [rad]")
grid on

subplot(3,1,2)
h=plot(t,omega); 
title("\omega [rad/s]")
grid on

subplot(3,1,3)
plot(t,alpha)
title("\alpha [rad/s^2]")
grid on

%pause(2)
% 
% figure()
% 
% plot(S2(1,:),S2(2,:))
% axis on
% hold on
% plot(x,y, "--r","LineWidth",1.4)
% xlim([-20,20])
% ylim([-20,20])
% title("trayectoria")
% axis equal
% hold off

%pause(2)
figure()
plot(t,Tm)
title("Torque de motor [Nm]")
grid on


workArea=[min([S1,S2],[],2);max([S1,S2],[],2)-min([S1,S2],[],2)];
%%
pause(1)

h=figure('units','normalized','outerposition',[0 0 1 1])

axis tight manual % this ensures that getframe() returns a consistent size
filename = 'Simulation.gif';

for k= 1:length(t)  
  subplot(1,2,1)
  plot([0,S1(1,k)],[0,S1(2,k)])
  hold on
  plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)])
  plot(c1(1,k),c1(2,k),"o")
  plot(c2(1,k)+S1(1,k),c2(2,k)+S1(2,k),"o")
  rectangle('Position',workArea,'FaceColor',[0, 0.1, 0,0.05])  
  plot(x,y,":")
  title("trayectoria")
  grid on
  axis equal
  xlim([-20,20])
  ylim([-20,20])
  hold off
  
  subplot(4,2,2)
  plot(t(1:k),Tm(:,1:k))
  title("Torque de motor [Nm]")
  grid on
  xlim([0,t(end)])
  ylim([min(Tm,[],"all"),max(Tm,[],"all")])
  
  subplot(4,2,4)
  plot(t(1:k),theta(:,1:k))
  title("\theta [rad]")
  xlim([0,t(end)])
  ylim([min(theta,[],"all"),max(theta,[],"all")])
  
  grid on

  subplot(4,2,6)
  plot(t(1:k),omega(:,1:k)); 
  title("\omega [rad/s]")
  xlim([0,t(end)])
  ylim([min(omega,[],"all"),max(omega,[],"all")])
  grid on

  subplot(4,2,8)
  plot(t(1:k),alpha(:,1:k))
  title("\alpha [rad/s^2]")
  xlim([0,t(end)])
  ylim([min(alpha,[],"all"),max(alpha,[],"all")])
  
  grid on

  drawnow
  
%   % Capture the plot as an image 
%   frame = getframe(h); 
%   im = frame2im(frame); 
%   [imind,cm] = rgb2ind(im,256); 
%   % Write to the GIF File 
%   if k == 1 
%       imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%   else 
%       imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dt); 
%   end 

end



%%

 

function rot=R(theta)
  rot=[cos(theta),-sin(theta);sin(theta),cos(theta)];
end






