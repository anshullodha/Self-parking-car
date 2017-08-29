clc
close all
clear all

generate_desired_Polynomials;

q = [xr_0; yr_0; th_0; phi_0; vel_0];
global Ku;
Ku=5;
t = 0;
time_span = 0:0.1:time(end);

dq = dynamics_car(t,q);

[time,states] = ode45(@dynamics_car,time_span,q);
%initializing
error(length(time),5)=zeros;
u1(length(time))=zeros;
u2(length(time))=zeros;
u1_d(length(time))=zeros;
u2_d(length(time))=zeros;
filename = 'Controller_tracking.gif';

for i=1:length(time)
        
    %error calculation
    xr_d(i) = ppval(pp_state.xr,time(i));
    yr_d(i) = ppval(pp_state.yr,time(i));
    th_d = ppval(pp_state.th,time(i));
    phi_d = ppval(pp_state.phi,time(i));
    vel_d = ppval(pp_state.vel,time(i));
    u1_d(i) = ppval(pp_controls.u1,time(i));
    u2_d(i) = ppval(pp_controls.u2,time(i));
    error(i,:)=[xr_d(i) yr_d(i) th_d phi_d vel_d]-states(i,:);
    
    [u1(i),u2(i)]=control_signal(time(i),states(i,:)');
end
for i=1:length(time)
    cla;
    plot(xr_d,yr_d,'r','linewidth',1.1)
    hold on;
    plot(states(:,1),states(:,2),'b--','linewidth',1.1)
    legend('Desired','Actual')
    plot_Environment;
    plotCLMR(states(i,1),states(i,2),states(i,3),states(i,4))
    pause(0.001)
    
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i  == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.01);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.01);
    end
    hold off
end
figure;
plot(time,states(:,1),time,states(:,2),time,states(:,3),time,states(:,4),time,states(:,5),'linewidth',1.5)
legend('x_r' ,'y_r','\theta','\phi','velocity')
xlabel('time')
ylabel('States')

figure;
subplot(3,1,1)
plot(time, u1,time, u1_d,'linewidth',1.5)
legend('control velocity_{commanded}' ,'control velocity_{desired}')
xlabel('time')
ylabel('control signal')

subplot(3,1,2)
plot(time, u2,time,u2_d,'linewidth',1.5)
legend('control steer_{commanded}' ,'control steer_{desired}')
xlabel('time')
ylabel('control signal')

subplot(3,1,3)
plot(time,error(:,1),'r',time,error(:,2),'b',time,error(:,3),'k--',time,error(:,4),'m',time,error(:,5),'c','linewidth',1.5);
legend('x_r' ,'y_r','\theta','\phi','velocity')
xlabel('time')
ylabel('error')