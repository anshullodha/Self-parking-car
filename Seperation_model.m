clc
close all
clear all

generate_desired_Polynomials;
q(:,1) = [xr_0; yr_0; th_0; phi_0; vel_0];
global Ku;
Ku=5;
t = 0;
dt = 0.1;
N=time(end)/dt;

C = [1 0 0 0 0;
    0 1 0 0 0];
   
R = diag([.01 .001]);

Q = diag([.01 .01]);
P_pl = eye(5);

X_hat_0 = q(:,1);
Y(:,1) = C*q(:,1);
X_hat(:,1)= X_hat_0;

for i = 1:round(N)
    t(i+1) = t(i)+dt;
    %generate control
    [u1(i),u2(i)]=control_signal(t(i),X_hat(:,i));
    
    % True process
    q(:,i+1)= q(:,i) + dt*[q(5,i)*cos(q(3,i))*cos(q(4,i));
                                    q(5,i)*sin(q(3,i))*cos(q(4,i));
                                    q(5,i)/15*sin(q(4,i));
                                    -u2(i);
                                    -Ku*(q(5,i)-u1(i))];
    
    Y(:,i+1) = C*q(:,i+1) + sqrt(R)*randn(size(C,1),1);  %*randn(2,1);
    
    % Observer model
    dFdx=[0 0 -X_hat(5,i)*sin(X_hat(3,i))*cos(X_hat(4,i)) -X_hat(5,i)*cos(X_hat(3,i))*sin(X_hat(4,i)) cos(X_hat(3,i))*cos(X_hat(4,i));
          0 0 X_hat(5,i)*cos(X_hat(3,i))*cos(X_hat(4,i)) -X_hat(5,i)*sin(X_hat(3,i))*sin(X_hat(4,i)) sin(X_hat(3,i))*cos(X_hat(4,i));
          0 0 0 X_hat(5,i)*cos(X_hat(4,i))/15 sin(X_hat(4,i));
          0 0 0 0 0;
          0 0 0 0 -Ku];
    
    dFdx_d = eye(5) + dt*dFdx;
    B = [0 0;
     0 0;
     0 0;
     0 -1;
     Ku 0];
    F=dt*B;
    
    P_mi = dFdx_d*P_pl*dFdx_d' + F*Q*F';
       
    X_hat(:,i+1) = X_hat(:,i) + dt*[X_hat(5,i)*cos(X_hat(3,i))*cos(X_hat(4,i));
                                    X_hat(5,i)*sin(X_hat(3,i))*cos(X_hat(4,i));
                                    X_hat(5,i)/15*sin(X_hat(4,i));
                                    -u2(i);
                                    -Ku*(X_hat(5,i)-u1(i))];
    Y_hat(:,i+1) = C*X_hat(:,i+1);
    % Update based on measurement
    e_Y  = Y(:,i+1) - Y_hat(:,i+1);
    S = C*P_mi*C'+R;
    K = P_mi*C'*inv(S);
    P_pl = (eye(5) - K*C)*P_mi;
    X_hat(:,i+1)=X_hat(:,i+1) + K*e_Y;
end
[u1(i+1),u2(i+1)]=control_signal(t(i),X_hat(:,i+1));
X_hat(:,end)
figure;
subplot(2,1,1)
plot(t,q(1,:),'r',t,X_hat(1,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('x_r')

subplot(2,1,2)
plot(t,q(2,:),'r',t,X_hat(2,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('y_r')

figure;
subplot(3,1,1)
plot(t,q(3,:),'r',t,X_hat(3,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('\theta (theta)')

subplot(3,1,2)
plot(t,q(4,:),'r',t,X_hat(4,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('\phi (phi)')

subplot(3,1,3)
plot(t,q(5,:),'r',t,X_hat(5,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('Velocity (v)')

error_desired;
