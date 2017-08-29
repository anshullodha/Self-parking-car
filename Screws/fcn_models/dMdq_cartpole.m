function p = dMdq_cartpole(th1_d,th2_d,th3_d,dth1_d,dth2_d,dth3_d,u2_d,u3_d,m1,m2,m3,l1,l2,l3,g)
%DMDQ_CARTPOLE
%    P = DMDQ_CARTPOLE(TH1_D,TH2_D,TH3_D,DTH1_D,DTH2_D,DTH3_D,U2_D,U3_D,M1,M2,M3,L1,L2,L3,G)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    20-Nov-2016 22:18:51

t2 = cos(th1_d);
t3 = sin(th2_d);
t4 = l1.*l2.*t2.*t3.*2.0;
t5 = cos(th2_d);
t6 = sin(th1_d);
t12 = l1.*l2.*t5.*t6.*2.0;
t7 = t4-t12;
t8 = m2.*t7;
t9 = cos(th3_d);
t10 = sin(th3_d);
t11 = l3.*t9.*th3_d;
t13 = l3.*t10;
t14 = t11+t13;
t15 = l3.*t9;
t20 = l3.*t10.*th3_d;
t16 = t15-t20;
t17 = l3.*t10.*2.0;
t18 = t11+t17;
t19 = l3.*t9.*2.0;
t21 = l1.*t2.*t14.*2.0;
t22 = t21-l1.*t6.*t16.*2.0;
t23 = m2.*t22.*(1.0./2.0);
t24 = l1.*t2.*t18.*2.0;
t25 = t19-t20;
t26 = l2.*t3.*t16.*2.0;
t27 = t26-l2.*t5.*t14.*2.0;
t28 = l2.*t5.*t18.*2.0;
t29 = t28-l2.*t3.*t25.*2.0;
p = reshape([0.0,t8,t23,t8,0.0,0.0,t23,0.0,m2.*(dth1_d.*l1.*t6.*t18.*2.0+dth1_d.*l1.*t2.*t25.*2.0).*(1.0./2.0),0.0,-t8,0.0,-t8,0.0,m2.*t27.*(-1.0./2.0),0.0,m2.*t27.*(-1.0./2.0),m2.*(dth2_d.*l2.*t3.*t18.*2.0+dth2_d.*l2.*t5.*t25.*2.0).*(1.0./2.0),0.0,0.0,m2.*(t24-l1.*t6.*t25.*2.0).*(-1.0./2.0),0.0,0.0,m2.*t29.*(-1.0./2.0),m2.*(t24-l1.*t6.*(t19-l3.*t10.*th3_d).*2.0).*(-1.0./2.0),m2.*t29.*(-1.0./2.0),m2.*(t16.*t18.*6.0-t14.*t25.*6.0-(t20-l3.*t9.*3.0).*(t11+dth1_d.*l1.*t2+dth2_d.*l2.*t5).*2.0+(t11+l3.*t10.*3.0).*(t20+dth2_d.*l2.*t3+dth1_d.*l1.*t6).*2.0).*(-1.0./2.0)],[3,3,3]);
