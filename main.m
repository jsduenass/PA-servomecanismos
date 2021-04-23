clear, clc, close all

%trayectoria
angle=linspace(0,2*pi,100);
t=angle;
L1=4;    L2=8;
r=8+2*cos(4*angle+pi);
[x,y]=pol2cart(angle,r);
% 
% x=(4+0.2*t).*cos(t);      y=(6-0.2*t).*sin(t);

L=sqrt(x.^2+y.^2);
alpha=atan2(y,x);

theta=[alpha-acos((L.^2+L1^2-L2^2)./(2*L*L1));
             acos((L.^2-L1^2-L2^2)/(2*L1*L2))];
 
omega=gradient(theta);

S1=[]; S2=[];

for k= 1:length(t)
  
  theta1=theta(1,k);
  theta2=theta(2,k);
  P1=L1*R(theta1)*[1;0];
  P2=L2*R(theta2+theta1)*[1;0]+P1;
  S1(:,end+1)=P1;
  S2(:,end+1)=P2;
  
end

figure ()
subplot(2,1,1)
plot(t,theta)
grid on

subplot(2,1,2)
h=plot(t,omega); 
grid on

pause(2)

figure()

plot(S2(1,:),S2(2,:))
axis on
hold on
plot(x,y, "--r")
xlim([-20,20])
ylim([-20,20])
hold off

pause(2)
%%
figure()
for k= 1:length(t)  
  plot([0,S1(1,k)],[0,S1(2,k)])
  hold on
  plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)])
 
  plot(x,y,"--")
  grid on
  
  axis equal
  xlim([-20,20])
  ylim([-20,20])
  hold off
  pause(0.1)
  
end


%%

 

function rot=R(theta)
  rot=[cos(theta),-sin(theta);sin(theta),cos(theta)];
end






