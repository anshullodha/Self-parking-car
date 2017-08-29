function p = dfdq_cartpole(th1_d,th2_d,th3_d,dth1_d,dth2_d,dth3_d,u2_d,u3_d,m1,m2,m3,l1,l2,l3,g)
%DFDQ_CARTPOLE
%    P = DFDQ_CARTPOLE(TH1_D,TH2_D,TH3_D,DTH1_D,DTH2_D,DTH3_D,U2_D,U3_D,M1,M2,M3,L1,L2,L3,G)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    20-Nov-2016 22:18:50

t2 = cos(th3_d);
t3 = sin(th3_d);
t4 = cos(th1_d);
t5 = l3.*t2.*2.0;
t18 = l3.*t3.*th3_d;
t6 = t5-t18;
t7 = sin(th1_d);
t8 = l3.*t3.*2.0;
t9 = l3.*t2.*th3_d;
t10 = t8+t9;
t11 = dth2_d.^2;
t12 = sin(th2_d);
t13 = l1.*l2.*t7.*t12.*2.0;
t14 = cos(th2_d);
t15 = l1.*l2.*t4.*t14.*2.0;
t16 = t13+t15;
t17 = th3_d.^2;
t24 = l3.*t2.*3.0;
t19 = t18-t24;
t20 = l3.*t3.*3.0;
t21 = t9+t20;
t22 = dth1_d.^2;
t23 = m2.*t16.*t22;
t25 = dth1_d.*l1.*t6.*t7.*2.0;
t30 = dth1_d.*l1.*t4.*t10.*2.0;
t26 = t25-t30;
t27 = dth1_d.*l1.*t4.*t21.*2.0;
t28 = dth1_d.*l1.*t7.*t19.*2.0;
t29 = t27+t28;
t31 = dth2_d.*l2.*t6.*t12.*2.0;
t36 = dth2_d.*l2.*t10.*t14.*2.0;
t32 = t31-t36;
t33 = dth2_d.*l2.*t14.*t21.*2.0;
t34 = dth2_d.*l2.*t12.*t19.*2.0;
t35 = t33+t34;
t56 = l3.*t2;
t37 = t18-t56;
t38 = l3.*t3;
t39 = t9+t38;
t40 = dth1_d.*l1.*t4.*t6.*2.0;
t41 = dth1_d.*l1.*t7.*t10.*2.0;
t42 = t40+t41;
t43 = m2.*t42.*(1.0./4.0);
t44 = m2.*t29.*th3_d.*(1.0./4.0);
t45 = l1.*t4.*t6.*2.0;
t46 = l1.*t7.*t10.*2.0;
t47 = t45+t46;
t48 = dth2_d.*l2.*t6.*t14.*2.0;
t49 = dth2_d.*l2.*t10.*t12.*2.0;
t50 = t48+t49;
t51 = m2.*t50.*(1.0./4.0);
t52 = m2.*t35.*th3_d.*(1.0./4.0);
t53 = l2.*t6.*t14.*2.0;
t54 = l2.*t10.*t12.*2.0;
t55 = t53+t54;
t57 = dth1_d.*l1.*t4;
t58 = dth2_d.*l2.*t14;
t59 = t9+t57+t58;
t60 = dth1_d.*l1.*t7;
t61 = dth2_d.*l2.*t12;
t62 = t18+t60+t61;
t63 = t19.*t59.*2.0;
t64 = t6.*t39.*6.0;
t65 = t10.*t37.*6.0;
t66 = t63+t64+t65-t21.*t62.*2.0;
p = reshape([-t17.*(m2.*t26.*(1.0./4.0)+m2.*t47.*(1.0./2.0))-m2.*t11.*t16+g.*l1.*m1.*t4+g.*l1.*m2.*t4+g.*l1.*m3.*t4,t23,th3_d.*(t44+dth1_d.*m2.*t26.*(1.0./4.0))+dth1_d.*(m2.*t26.*th3_d.*(1.0./4.0)-dth1_d.*m2.*(l1.*t4.*t37.*2.0-l1.*t7.*t39.*2.0).*(1.0./2.0)),m2.*t11.*t16,-t23-t17.*(m2.*t32.*(1.0./4.0)+m2.*t55.*(1.0./2.0))+g.*l2.*m2.*t14+g.*l2.*m3.*t14,th3_d.*(t52+dth2_d.*m2.*t32.*(1.0./4.0))+dth2_d.*(m2.*t32.*th3_d.*(1.0./4.0)+dth2_d.*m2.*(l2.*t12.*t39.*2.0-l2.*t14.*t37.*2.0).*(1.0./2.0)),-t17.*(m2.*t29.*(1.0./4.0)+m2.*(l1.*t4.*t19.*2.0-l1.*t7.*t21.*2.0).*(1.0./2.0))+th3_d.*(t43-m2.*(l1.*t6.*t7.*2.0-l1.*t4.*t10.*2.0).*(1.0./2.0)).*2.0,-t17.*(m2.*t35.*(1.0./4.0)-m2.*(l2.*t12.*t21.*2.0-l2.*t14.*t19.*2.0).*(1.0./2.0))+th3_d.*(t51-m2.*(l2.*t6.*t12.*2.0-l2.*t10.*t14.*2.0).*(1.0./2.0)).*2.0,-dth1_d.*(t43-t44+dth1_d.*m2.*t47.*(1.0./2.0))-dth2_d.*(t51-t52+dth2_d.*m2.*t55.*(1.0./2.0))-th3_d.*(m2.*t66.*(1.0./4.0)+m2.*th3_d.*(t19.*t37.*-8.0-t21.*t39.*8.0+t6.^2.*6.0+t10.^2.*6.0+t59.*(t9+l3.*t3.*4.0).*2.0+t62.*(t18-l3.*t2.*4.0).*2.0).*(1.0./4.0)-dth1_d.*m2.*t29.*(1.0./4.0)-dth2_d.*m2.*t35.*(1.0./4.0))-dth1_d.*m2.*t42.*(1.0./4.0)-dth2_d.*m2.*t50.*(1.0./4.0)-m2.*t66.*th3_d.*(1.0./4.0)+g.*l3.*m3.*t2],[3,3]);
