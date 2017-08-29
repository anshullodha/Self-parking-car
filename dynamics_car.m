function dq = dynamics_car(t,q)
global Ku;
th = q(3);
phi=q(4);
vel=q(5);

[u1,u2]=control_signal(t,q);

dxr = vel*cos(th)*cos(phi);
dyr = vel*sin(th)*cos(phi);
dth = vel*sin(phi)/15;
dphi = -u2;
dvel = -Ku*(vel-u1);

dq = [dxr; dyr; dth; dphi; dvel];
end