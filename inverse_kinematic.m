function theta=inverse_kinematic(x,y) 
global L1  L2
[angle,r]=cart2pol(x,y);
L=sqrt(x.^2+y.^2);

% perfil de movimiento
theta=[angle-acos((L.^2+L1^2-L2^2)./(2*L*L1));
             acos((L.^2-L1^2-L2^2)/(2*L1*L2))];
end
