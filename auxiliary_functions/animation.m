%%
pause(1)

%h=figure()
%h=figure('units','normalized','outerposition',[0 0 1 1])   %full screen

h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

axis tight manual % this ensures that getframe() returns a consistent size
filename = './media/inverseKinematics.gif';
capture=false;


for k= 1:length(t)  
  subplot(2,2,1)
  %arm
  plot([0,S1(1,k)],[0,S1(2,k)], "LineWidth",4)
  hold on
  plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)], "LineWidth",2)
  plot(c1(1,k),c1(2,k),"ob")
  plot(c2(1,k)+S1(1,k),c2(2,k)+S1(2,k),"or")
  
  % work area
  %rectangle('Position',workArea,"LineStyle","--")  
  xline(limit_x,"--")
  yline(limit_y,"--")
  %target trajectory
  plot(x,y)
  
  title("trayectoria")
  grid on
  axis equal
  xlim([-14,max(x)+2])
  ylim([-20,max(y)+2])
  hold off
  
  
  subplot(3,2,5)
  plot(t(1:k),Tm(:,1:k))
  title("Torque de motor [Nm]")
  grid on
  xlim([0,t(end)])
  ylim([min(Tm,[],"all"),max(Tm,[],"all")])
  
  subplot(3,2,2)
  plot(t(1:k),theta_m(:,1:k))
  title("\theta_m [rad]")
  xlim([0,t(end)])
  ylim([min(theta_m,[],"all"),max(theta_m,[],"all")])
  grid on

  plot(t(1:k),velocity(1:k))
  title("velocidad [cm/s]")
  ylim([0,12])
  xlim([0,t(end)])
  
  
  subplot(3,2,4)
  plot(t(1:k),omega_m(:,1:k)); 
  title("\omega_m [rad/s]")
  xlim([0,t(end)])
  ylim([min(omega_m,[],"all"),max(omega_m,[],"all")])
  grid on

  subplot(3,2,6)
  plot(t(1:k),alpha_m(:,1:k))
  title("\alpha_m [rad/s^2]")
  xlim([0,t(end)])
  ylim([min(alpha_m,[],"all"),max(alpha_m,[],"all")])
  
  grid on

  drawnow
  n=5;
  if (capture & mod(k,n)==1)
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',n*dt); 
    end 
  end

end

