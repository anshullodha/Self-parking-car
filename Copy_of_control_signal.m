function [u1,u2]=control_signal(t,q)
global Ku;
generate_desired_Polynomials;
u1_d = ppval(pp_controls.u1,t);
u2_d = ppval(pp_controls.u2,t);
xr_d = ppval(pp_state.xr,t);
yr_d = ppval(pp_state.yr,t);
th_d = ppval(pp_state.th,t);
phi_d = ppval(pp_state.phi,t);
vel_d = ppval(pp_state.vel,t);
q_d = [xr_d; yr_d; th_d; phi_d; vel_d];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Linearization of the system
A=[0 0 -vel_d*sin(th_d)*cos(phi_d) -vel_d*cos(th_d)*sin(phi_d) cos(th_d)*cos(phi_d);
    0 0 vel_d*cos(th_d)*cos(phi_d) -vel_d*sin(th_d)*sin(phi_d) sin(th_d)*cos(phi_d);
    0 0 0 vel_d*cos(phi_d)/15 sin(phi_d);
    0 0 0 0 0;
    0 0 0 0 -Ku];
B = [0 0;
     0 0;
     0 0;
     0 -1;
     Ku 0];
M=ctrb(A,B);%full rank, system is controllable
%p = [-1.5;-2;-0.8;-0.8;-0.7];%high value for y-coordinate as critical parameter
p = [-3;-4;-5;-6;-7];
if rank(M)==5
    K = place(A,B,p);
else
    K=zeros(2,5);
end

%Implementing High Gain Controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (t>time(end))
    u1_d=0;
    u2_d=0;
end
u1= -K(1,:)*(q -q_d)  + u1_d;
u2= -K(2,:)*(q -q_d) + u2_d;
%apply contraints
if(norm(u1)>12)% Actuator contraints
    u1=sign(u1)*12;
end
if(norm(u2)>0.785398)% steering constraints
    u2=sign(u2)*0.785398;
end

end