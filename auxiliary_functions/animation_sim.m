

filename = './Simulation.gif';
capture=false;
h=figure('Renderer', 'painters', 'Position', [100 100 700 400]);

for k= 1:5:length(time)
  
subplot(1,2,1)
plot(S2_ref(1,1:k),S2_ref(2,1:k),"--r")
hold on

plot([0,S1(1,k)],[0,S1(2,k)], "b","LineWidth",4)
plot([S1(1,k), S2(1,k)],[S1(2,k), S2(2,k)],"r", "LineWidth",2)

plot(S2(1,1:k),S2(2,1:k),"g")

plot(S2(1,k),S2(2,k),"og")

xline(limit_x,"--")
yline(limit_y,"--")

axis equal
ylim([-40,40]);

xlim([-40,40]);

title("trayectoria")
hold off


subplot(3,2,2)
style={"--","--","-","-"};
color={"blue","red","blue","red"};

g=plot(time,theta_ref,time,theta_traveled);

[g(:).LineStyle] = style{:};
[g(:).Color] = color{:};

ylabel("\theta  [rad]")
xlabel("tiempo[s]")
hold off

subplot(3,2,4)
plot(time,theta_traveled-theta_ref)
yline(0)
ylabel("Error [rad]")
xlabel("tiempo[s]")

subplot(3,2,6)
plot(time(1:k),torque(1:k,:))
xlim([0,time(end)])
ylim([min(torque,[],'all'),max(torque,[],'all')])

ylabel("Torque [Nm]")
xlabel("tiempo[s]")
  drawnow
  if (capture & mod(k,20)==1)
   
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',16*n*dt); 
    end 
  end

end
