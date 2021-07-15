Lbarra1=0.22;
Lbarra2=0.2;
masaL1=0.455*Lbarra1;   
masaL2=0.455*Lbarra2;
masaCarga=0.02;

masaM1=0.4;
masaM2=0.2;

masa1=masaL1+masaM2;
masa2=masaL2+masaCarga;

lc_1=(masaL1*Lbarra1/2+masaM2*Lbarra1)/(masaL1+masaM2);
lc_2=(masaL2*Lbarra2/2+masaCarga*Lbarra2)/(masaL2+masaCarga);

I1=(1/12)*masa1*Lbarra1^2+masa1*(lc_1-Lbarra1/2)^2;

I2=(1/12)*masa2*Lbarra2^2+masa2*(lc_2-Lbarra2/2)^2;

A=masa1*lc_1^2+I1+masa2*Lbarra1^2+masa2*lc_2^2+I2+2*masa2*Lbarra1*lc_2.*cos(theta_m(2,:));
B=masa2*lc_2^2+I2+masa2*Lbarra1*lc_2.*cos(theta_m(2,:));
D=2*masa2*Lbarra1*lc_2.*sin(theta_m(2,:));
E=2*masa2*Lbarra1*lc_2.*sin(theta_m(2,:));

F=masa2*lc_2^2+masa2*Lbarra1*lc_2*cos(theta_m(2,:))+I2;
H=masa2*lc_2^2+I2;
Nmat=masa2*Lbarra1*lc_2*sin(theta_m(2,:));

Tmotor1=A.*alpha_m(1,:)+B.*alpha_m(2,:)+D.*omega_m(1,:).*omega_m(2,:)+E.*omega_m(2,:).^2+...
    9.81*((masa1*lc_1+masa2*Lbarra1)*cos(theta_m(1,:))+masa2*lc_2*cos(theta_m(1,:)+theta_m(2,:)));
Tmotor2=F.*alpha_m(1,:)+H.*alpha_m(2,:)+Nmat.*omega_m(1,:).*omega_m(2,:)...
    -(-masa2*Lbarra1*lc_2.*sin(theta_m(2,:)).*omega_m(1,:).*(omega_m(1,:)+omega_m(2,:)))...
    +masa2*9.81*lc_2.*cos(theta_m(1,:)+theta_m(2,:));
%%
  close all
  subplot(2,1,1)
  plot(t,Tmotor1,t, -Tm(1,:))
  legend(["Lagrangian","Newtonian"])
  title("Torque Motor 1")
  subplot(2,1,2)
  plot(t,Tmotor2,t, -Tm(2,:))
  legend(["Lagrangian","Newtonian"])
  title("Torque Motor 2")
  
  figure()
  subplot(2,1,1)
  plot(t,[alpha_m(1,:).*J_L(1) ; Tg(1,:); Tf(1,:)])
  title("Torque Motor 1")
  legend(["T inercial","T gravitacional","T friccion"])
 
  subplot(2,1,2)
  plot(t,[alpha_m(2,:).*J_L(2) ; Tg(2,:); Tf(2,:)])
  legend(["T inercial","T gravitacional","T friccion"])
  title("Torque Motor 2")
  
  
  
