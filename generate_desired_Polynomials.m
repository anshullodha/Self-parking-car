load solution.mat

time = solution.phase.time;
state = solution.phase.state;
controls = solution.phase.control;

pp_xr = spline(time,state(:,1));
pp_yr = spline(time,state(:,2));
pp_th = spline(time,state(:,3));
pp_phi = spline(time,state(:,4));
pp_vel = spline(time,state(:,5));
pp_u1 = spline(time,controls(:,1));
pp_u2 = spline(time,controls(:,2));

pp_state.xr = pp_xr;
pp_state.yr = pp_yr;
pp_state.th = pp_th;
pp_state.phi = pp_phi;
pp_state.vel = pp_vel;
pp_controls.u1 = pp_u1;
pp_controls.u2 = pp_u2;
