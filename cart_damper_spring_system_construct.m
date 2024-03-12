%% Compare predictive estimates for nonlinear terms using different assumptions
clear; clc; close all;

L = 129; % Simulated step size

%% Set the coefficients of the system
A=[1 0.3;0 0.8992];
B=[0;0.24];

%% Set parameters for the model predictive control
Np=8;% Predicted increment size
Q=0.5*eye(size(A,1)); R=0.3*eye(size(B,2));% Optimized parameters for the target, utilizing a weighted matrix.

%% The process involves calculating the weighted matrix and superimposing the derived equation matrix.
At=[]; Bt=[]; temp=[];% Transformed into a matrix of derived equations concerning state variables represented in relation to control variables ut
Qt=[]; Rt=[];         % The matrix with weights following the conversion
Ft=[];Zt=zeros(size(A,1));% Nonlinearities are characterized by the consideration of incremental matrices
for i=1:Np
    At=[At; A^i];
    Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i-1)*B temp];
    temp=[A^(i-1)*B temp];
    Zt = Zt+A^(i-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
    Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
end

%% Establishing attack parameters
tao_x = 6;          % Sensor-to-controller delay
tao_u = 4;          % Delay from the controller to the actuator
dec_rou_y = 0.2;    % Probability of Output deception Attack
dec_rou_u = 0.25;   % Probability of input deception Attack
DoS_rou_y = 0.15;   % Probability of Output DoS Attack
DoS_rou_u = 0.1;    % Probability of input DoS Attack
% tao_xk = ones(1,L); % Delay of DoS Attack from Sensor to Controller
% tao_uk = ones(1,L); % Delay of DoS Attack from Controller to Actuator

%% 攻击初始化
% s_c_decattack = zeros(1,L);% Sensor-to-controller deception attack sequence
% c_a_decattack = zeros(1,L);% Controller-to-actuator deception attack sequence
% s_c_DoSattack = zeros(1,L);% Sensor-to-controller DoS attack sequence
% c_a_DoSattack = zeros(1,L);% Controller-to-actuator DoS attack sequence
wx = zeros(1,L);           % Sensor-to-controller deception attack disturbance sequence
wu = zeros(1,L);           % Controller-to-actuator deception attack disturbance sequence

%% 攻击序列
for k=1:L
%     s_c_decattack(k) = randsrc(1,1,[1,0;dec_rou_y,1-dec_rou_y]);
%     c_a_decattack(k) = randsrc(1,1,[1,0;dec_rou_u,1-dec_rou_u]);
%     
%     s_c_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_y,1-DoS_rou_y]);
%     c_a_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_u,1-DoS_rou_u]);
% 
%     tao_xk(1,k) = round(rand(1,1)*(tao_x-1))+1;
%     tao_uk(1,k) = round(rand(1,1)*(tao_u-1))+1;

    wx(:,k) = 0.5*sin(k);
    wu(:,k) = 0.1*cos(k);
end

load("c_a_decattack.mat")% Controller-to-actuator deception attack sequence
load("c_a_DoSattack.mat")% Controller-to-actuator DoS attack sequence
load("s_c_decattack.mat")% Sensor-to-controller deception attack sequence
load("s_c_DoSattack.mat")% Sensor-to-controller DoS attack sequence
load("tao_uk.mat")       % Delay of DoS Attack from Controller to Actuator
load("tao_xk.mat")       % Delay of DoS Attack from Sensor to Controller

%% proposed method
%% Assume that the nonlinear term is basically constant in the prediction domain
x(:,1)=[-1.2;1.2];            % Initial values of system status parameters
u(:,1)=0;                     % Initial value of the actuator parameter
xp(:,1) = [-1.2;1.2];         % Initial value of the discriminant model
f(:,1) = f_function(x(:,1),1);% The original value of the nonlinear term
e(:,1) = [0;0];               % The initial value of the trigger error
kexi(:,1) = 0;                % The initial value of the controller output
xk = zeros(2,tao_x);          % The previous state of the system status
ek = zeros(2,tao_x);          % The previous value of the trigger error
kexik = zeros(1,tao_u);       % The previous value of the controller output
wxk = zeros(2,tao_x);         % The previous value of the disturbance from the sensor to the controller
wuk = zeros(1,tao_u);         % Past disturbance from the controller to the actuator
xjk = zeros(2,1);             % The status output value at the most recent triggering moment
J = zeros(1,L);               % Sequence of performance indices

%% Dynamic event-triggered
zeta = 1;                     % Momentum factors induced by dynamic events ζ
theta = 0.5;                  % Formula for iterating the momentum factor θ   
deta = 6;                     % Dynamic events activate iterative elements δ
eipilon = 0.01;               % Threshold for triggering
event = zeros(1,L);           % Sequence of triggers
sum_e = 0;                    % An error in the synthetic state has been identified

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk-x(:,k),2)^2 >= eipilon+1/deta*zeta
        xjk = x(:,k);                              % Update the status of the most recent trigger
        e(:,k) = xjk-x(:,k);
        x_n = (1-s_c_DoSattack(k))*(x(:,k)+e(:,k)+s_c_decattack(k)*(-2*x(:,k)-2*e(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk(:,tao_xk(k))+ek(:,tao_xk(k))+s_c_decattack(k)*(-2*xk(:,tao_xk(k)) ...
            -2*ek(:,tao_xk(k))+wxk(:,tao_xk(k)))); % The value obtained by the detection mechanism
        if norm((xp(:,k)-x_n),2)^2 >= 0.5          % Evaluation of the detection mechanism
            xp(:,k) = xp(:,k);
        else
            xp(:,k) = x_n;
        end

        %% Controller section  
        ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp(:,k)+Ft*f_function(xp(:,k),k)); 
        kexi(k) = ut(1);                                       % Controller output       
        xp(:,k+1) = A*xp(:,k)+B*kexi(k)+f_function(xp(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u(:,k) = (1-c_a_DoSattack(k))*(kexi(k)+c_a_decattack(k)*(-kexi(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik(tao_uk(k))+c_a_decattack(k)*(-kexik(tao_uk(k))+wuk(tao_uk(k))));

        event(k) = 1;
    else
        event(k) = 0;
        e(:,k) = xjk-x(:,k);
        kexi(k) = ut(1); 
        xp(:,k+1) = A*xp(:,k)+B*kexi(k)+f_function(xp(:,k),k);
        u(:,k) = u(:,k-1);
    end

    J(k) = x(:,k)'*Q*x(:,k)+u(k)'*R*u(k);           % Calculation of the performance index
    x(:,k+1) = A*x(:,k) + B*u(k)+f_function(x(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e = sum_e+x(:,k)'*x(:,k);                   % Total calculation error
    zeta = theta*zeta+eipilon-norm(xjk-x(:,k),2)^2; % Formula for iterating the momentum factor

    %% Data update
    for i=tao_x:-1:2
        xk(:,i) = xk(:,i-1);
        ek(:,i) = ek(:,i-1);
        wxk(:,i) = wxk(:,i-1);
    end
    xk(:,1) = x(:,k);
    ek(:,1) = e(:,k);
    wxk(:,1) = wx(:,k);
    for i=tao_u:-1:2
        kexik(i) = kexik(i-1);
        wuk(:,i) = wuk(:,i-1);
    end
    kexik(1) = kexi(:,k);
    wuk(:,1) = wu(:,k);
    
end

sum_t=L/sum(event);
sum_J = sum(J);% Calculation of the performance index

%% proposed method
%% Assume that the nonlinear term utilizes the controller output value of the previous time in the prediction domain
x1(:,1)=[-1.2;1.2];            % Initial values of system status parameters
u1(:,1)=0;                     % Initial value of the actuator parameter
xp1(:,1) = [-1.2;1.2];         % Initial value of the discriminant model
f1(:,1) = f_function(x1(:,1),1);% The original value of the nonlinear term
e1(:,1) = [0;0];               % The initial value of the trigger error
kexi1(:,1) = 0;                % The initial value of the controller output
xk1 = zeros(2,tao_x);          % The previous state of the system status
ek1 = zeros(2,tao_x);          % The previous value of the trigger error
kexik1 = zeros(1,tao_u);       % The previous value of the controller output
wxk1 = zeros(2,tao_x);         % The previous value of the disturbance from the sensor to the controller
wuk1 = zeros(1,tao_u);         % Past disturbance from the controller to the actuator
xjk1 = zeros(2,1);             % The status output value at the most recent triggering moment
J1 = zeros(1,L);               % Sequence of performance indices

%% The process involves calculating the weighted matrix and superimposing the derived equation matrix.
At=[]; Bt=[]; temp=[];% Transformed into a matrix of derived equations concerning state variables represented in relation to control variables ut
Qt=[]; Rt=[];         % The matrix with weights following the conversion
Ft=[]; tempf=[]; Ftf=[]; Zt=zeros(size(A,1));% Nonlinearities are characterized by the consideration of incremental matrices
for i=1:Np
    At=[At; A^i];
    Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i-1)*B temp];
    temp=[A^(i-1)*B temp];
    Ftf=[Ftf zeros(size(Ftf,1),size(eye(2),2));A^(i-1)*eye(2) tempf];
    tempf=[A^(i-1)*eye(2) tempf];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
    Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
end

%% Dynamic event-triggered
zeta1 = 1;                     % Momentum factors induced by dynamic events ζ
theta1 = 0.5;                  % Formula for iterating the momentum factor θ   
deta1 = 6;                     % Dynamic events activate iterative elements δ
eipilon1 = 0.01;               % Threshold for triggering
event1 = zeros(1,L);           % Sequence of triggers
sum_e1 = 0;                    % An error in the synthetic state has been identified

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk1-x1(:,k),2)^2 >= eipilon1+1/deta1*zeta1
        xjk1 = x1(:,k);                              % Update the status of the most recent trigger
        e1(:,k) = xjk1-x1(:,k);
        x_n = (1-s_c_DoSattack(k))*(x1(:,k)+e1(:,k)+s_c_decattack(k)*(-2*x1(:,k)-2*e1(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk1(:,tao_xk(k))+ek1(:,tao_xk(k))+s_c_decattack(k)*(-2*xk1(:,tao_xk(k)) ...
            -2*ek1(:,tao_xk(k))+wxk1(:,tao_xk(k)))); % The value obtained by the detection mechanism
        if norm((xp1(:,k)-x_n),2)^2 >= 0.5          % Evaluation of the detection mechanism
            xp1(:,k) = xp1(:,k);
        else
            xp1(:,k) = x_n;
        end

        %% Controller section  
        ut1 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp1(:,k)+Ftf*estimate_f(xp1(:,k),kexik1(1),A,B,k,Np)); 
        kexi1(k) = ut1(1);                                       % Controller output       
        xp1(:,k+1) = A*xp1(:,k)+B*kexi1(k)+f_function(xp1(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u1(:,k) = (1-c_a_DoSattack(k))*(kexi1(k)+c_a_decattack(k)*(-kexi1(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik1(tao_uk(k))+c_a_decattack(k)*(-kexik1(tao_uk(k))+wuk1(tao_uk(k))));

        event1(k) = 2;
    else
        event1(k) = 0;
        e1(:,k) = xjk1-x1(:,k);
        kexi1(k) = ut1(1); 
        xp1(:,k+1) = A*xp1(:,k)+B*kexi1(k)+f_function(xp1(:,k),k);
        u1(:,k) = u1(:,k-1);
    end

    J1(k) = x1(:,k)'*Q*x1(:,k)+u1(k)'*R*u1(k);           % Calculation of the performance index
    x1(:,k+1) = A*x1(:,k) + B*u1(k)+f_function(x1(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e1 = sum_e1+x1(:,k)'*x1(:,k);                   % Total calculation error
    zeta1 = theta1*zeta1+eipilon1-norm(xjk1-x1(:,k),2)^2; % Formula for iterating the momentum factor

    %% Data update
    for i=tao_x:-1:2
        xk1(:,i) = xk1(:,i-1);
        ek1(:,i) = ek1(:,i-1);
        wxk1(:,i) = wxk1(:,i-1);
    end
    xk1(:,1) = x1(:,k);
    ek1(:,1) = e1(:,k);
    wxk1(:,1) = wx(:,k);
    for i=tao_u:-1:2
        kexik1(i) = kexik1(i-1);
        wuk1(:,i) = wuk1(:,i-1);
    end
    kexik1(1) = kexi1(:,k);
    wuk1(:,1) = wu(:,k);
    
end

sum_t1 = 2*L/sum(event1);
sum_J1 = sum(J1);% Calculation of the performance index

%% proposed method
%% Assume that the nonlinear term uses only the predicted value of the state in the prediction domain
x2(:,1)=[-1.2;1.2];            % Initial values of system status parameters
u2(:,1)=0;                     % Initial value of the actuator parameter
xp2(:,1) = [-1.2;1.2];         % Initial value of the discriminant model
f2(:,1) = f_function(x2(:,1),1);% The original value of the nonlinear term
e2(:,1) = [0;0];               % The initial value of the trigger error
kexi2(:,1) = 0;                % The initial value of the controller output
xk2 = zeros(2,tao_x);          % The previous state of the system status
ek2 = zeros(2,tao_x);          % The previous value of the trigger error
kexik2 = zeros(1,tao_u);       % The previous value of the controller output
wxk2 = zeros(2,tao_x);         % The previous value of the disturbance from the sensor to the controller
wuk2 = zeros(1,tao_u);         % Past disturbance from the controller to the actuator
xjk2 = zeros(2,1);             % The status output value at the most recent triggering moment
J2 = zeros(1,L);               % Sequence of performance indices

%% Dynamic event-triggered
zeta2 = 1;                     % Momentum factors induced by dynamic events ζ
theta2 = 0.5;                  % Formula for iterating the momentum factor θ   
deta2 = 6;                     % Dynamic events activate iterative elements δ
eipilon2 = 0.01;               % Threshold for triggering
event2 = zeros(1,L);           % Sequence of triggers
sum_e2 = 0;                    % An error in the synthetic state has been identified

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk2-x2(:,k),2)^2 >= eipilon2+1/deta2*zeta2
        xjk2 = x2(:,k);                              % Update the status of the most recent trigger
        e2(:,k) = xjk2-x2(:,k);
        x_n = (1-s_c_DoSattack(k))*(x2(:,k)+e2(:,k)+s_c_decattack(k)*(-2*x2(:,k)-2*e2(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk2(:,tao_xk(k))+ek2(:,tao_xk(k))+s_c_decattack(k)*(-2*xk2(:,tao_xk(k)) ...
            -2*ek2(:,tao_xk(k))+wxk2(:,tao_xk(k)))); % The value obtained by the detection mechanism
        if norm((xp2(:,k)-x_n),2)^2 >= 0.5          % Evaluation of the detection mechanism
            xp2(:,k) = xp2(:,k);
        else
            xp2(:,k) = x_n;
        end

        %% Controller section  
        ut2 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp2(:,k)+Ftf*estimate_f1(xp2(:,k),A,k,Np)); 
        kexi2(k) = ut2(1);                                       % Controller output       
        xp2(:,k+1) = A*xp2(:,k)+B*kexi2(k)+f_function(xp2(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u2(:,k) = (1-c_a_DoSattack(k))*(kexi2(k)+c_a_decattack(k)*(-kexi2(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik2(tao_uk(k))+c_a_decattack(k)*(-kexik2(tao_uk(k))+wuk2(tao_uk(k))));

        event2(k) = 3;
    else
        event2(k) = 0;
        e2(:,k) = xjk2-x2(:,k);
        kexi2(k) = ut2(1); 
        xp2(:,k+1) = A*xp2(:,k)+B*kexi2(k)+f_function(xp2(:,k),k);
        u2(:,k) = u2(:,k-1);
    end

    J2(k) = x2(:,k)'*Q*x2(:,k)+u2(k)'*R*u2(k);           % Calculation of the performance index
    x2(:,k+1) = A*x2(:,k) + B*u2(k)+f_function(x2(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e2 = sum_e2+x2(:,k)'*x2(:,k);                   % Total calculation error
    zeta2 = theta2*zeta2+eipilon2-norm(xjk2-x2(:,k),2)^2; % Formula for iterating the momentum factor

    %% Data update
    for i=tao_x:-1:2
        xk2(:,i) = xk2(:,i-1);
        ek2(:,i) = ek2(:,i-1);
        wxk2(:,i) = wxk2(:,i-1);
    end
    xk2(:,1) = x2(:,k);
    ek2(:,1) = e2(:,k);
    wxk2(:,1) = wx(:,k);
    for i=tao_u:-1:2
        kexik2(i) = kexik2(i-1);
        wuk2(:,i) = wuk2(:,i-1);
    end
    kexik2(1) = kexi2(:,k);
    wuk2(:,1) = wu(:,k);
    
end

sum_t2 = 3*L/sum(event2);
sum_J2 = sum(J2);% Calculation of the performance index

%% Simulation outcome
figure(1)
subplot(2,1,1)
i=1:L+1;
plot(i,x1(1,i),'--b',i,x2(1,i),'--r',i,x(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{1,k}');
% title('State output of comparison');
legend('Assumption 2 x_{1,k}','Assumption 3 x_{1,k}','Proposed method x_{1,k}');
grid on;
axis([0 L -2 3]);
subplot(2,1,2)
i=1:L+1;
plot(i,x1(2,i),'--b',i,x2(2,i),'--r',i,x(2,i),'k','linewidth',2);
xlabel('k/step');
ylabel('x_{2,k}');
legend('Assumption 2 x_{2,k}','Assumption 3 x_{2,k}','Proposed method x_{2,k}');
grid on;
axis([0 L -2 3]);

figure(2)
subplot(4,1,1)
i=1:L;
b=bar(i,s_c_decattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\alpha_{k}^x');
% title('s-c deception attack');
grid on;
subplot(4,1,2)
i=1:L;
b=bar(i,s_c_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\beta_{k}^x');
% title('s-c DoS attack');
grid on;
subplot(4,1,3)
i=1:L;
b=bar(i,c_a_decattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\alpha_{k}^u');
% title('c-a deception attack');
grid on;
subplot(4,1,4)
i=1:L;
b=bar(i,c_a_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
xlabel('k/step');
ylabel('\beta_{k}^u');
% title('c-a DoS attack');
grid on;

figure(3)
subplot(2,1,1)
i=1:L;
plot(i,u1(1,i),'--b',i,u2(1,i),'--r',i,u(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('u_k');
% title('Control law comparison');
legend('Assumption 2 u_k','Assumption 3 u_k','Proposed method u_k');
grid on;
axis([0 L -3 3]);
subplot(2,1,2)
i=1:L;
sz = 50;
scatter(i,event2,sz,'s');
hold on;
scatter(i,event1,sz,'d');
hold on;
scatter(i,event,sz,'h');
hold on;
xlabel('k/step');
% ylabel('The triggering instants');
yticks([1 2 3])
yticklabels({'Assumption 2','Assumption 3','Proposed method'})
grid on;
axis([0 L 1 3]);


