function dq_all = get_dynamics(xr_all,yr_all,th_all,phi_all,vel_all,u1_all,u2_all)


for i = 1:length(xr_all)
    
    xr = xr_all(i);    
    yr = yr_all(i);
    th = th_all(i);
    phi = phi_all(i);
    vel = vel_all(i);
    dxr = vel*cos(th)*cos(phi);
    dyr = vel*sin(th)*cos(phi);
    dth = vel*sin(phi)/15;
    dphi = -u2_all(i);
    Ku=5;
    dvel= -Ku*(vel-u1_all(i));
    %ff = ff_3R(xr,yr,th,dxr,dyr,dth,m1,m2,m3,l1,l2,l3,u2_f,u3_f,g);
    %M = M_3R(xr,yr,th,dxr,dyr,dth,m1,m2,m3,l1,l2,l3,u2_f,u3_f,g);


    %ddq = M\ff;
    
    dq_all(i,:) = [dxr dyr dth dphi dvel];
end