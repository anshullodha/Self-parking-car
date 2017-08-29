function phaseout = CLMR_continuous(input)


xr_all   = input.phase.state(:,1);
yr_all  = input.phase.state(:,2);
th_all  = input.phase.state(:,3);
phi_all = input.phase.state(:,4);
vel_all = input.phase.state(:,5);
u1_all   = input.phase.control(:,1);
u2_all   = input.phase.control(:,2);

dq_all = get_dynamics(xr_all,yr_all,th_all,phi_all,vel_all,u1_all,u2_all);


phaseout.dynamics  = dq_all;
phaseout.integrand = u2_all.^2;