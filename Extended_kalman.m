clc
close all
clear all

load('tracking.mat')

dt = 0.01;

theta = pi/2;


C = [1 0 0 0 0;
    0 1 0 0 0];
    %0 0 0 1 0;
    %0 0 0 0 1];
R = diag([.01 .01]);%0.1 0.1]);

Q = diag([.01 .01]);
P_pl = eye(5);

X=states';
X_hat_0 = X(:,1);

Y(:,1) = C*X(:,1);
X_hat(:,1)= X_hat_0;

for i = 1:497

    Y(:,i+1) = C*X(:,i+1) + sqrt(R)*randn(size(C,1),1);  %*randn(2,1);
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
%X_hat(3,1) = X_hat(3,end);
P_pl = P_mi;
t=time;
figure;

subplot(2,1,1)
plot(t,X(1,:),'r',t,X_hat(1,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('x_r')

subplot(2,1,2)
plot(t,X(2,:),'r',t,X_hat(2,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('y_r')

figure;
subplot(3,1,1)
plot(t,X(3,:),'r',t,X_hat(3,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('\theta (theta)')

subplot(3,1,2)
plot(t,X(4,:),'r',t,X_hat(4,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('\phi (phi)')

subplot(3,1,3)
plot(t,X(5,:),'r',t,X_hat(5,:),'k--','Linewidth',1.2)
legend('true','estimate')
xlabel('time')
ylabel('Velocity (v)')



    