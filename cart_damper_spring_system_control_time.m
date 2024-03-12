%% The influence of control domain selection on the system is compared between prediction domain and control domain
clear; clc; close all;

L = 129; % Simulated step size

%% Set the coefficients of the system
A=[1 0.3;0 0.8992];
B=[0;0.24];

%% Set parameters for the model predictive control
Np=8;% Predicted increment size
Q=0.5*eye(2); R=0.3;% Optimized parameters for the target, utilizing a weighted matrix.

%% Establishing attack parameters
tao_x = 6;          % Sensor-to-controller delay
tao_u = 4;          % Delay from the controller to the actuator
dec_rou_y = 0.2;    % Probability of Output deception Attack
dec_rou_u = 0.25;   % Probability of input deception Attack
DoS_rou_y = 0.15;   % Probability of Output DoS Attack
DoS_rou_u = 0.1;    % Probability of input DoS Attack
tao_xk = ones(1,L); % Delay of DoS Attack from Sensor to Controller
tao_uk = ones(1,L); % Delay of DoS Attack from Controller to Actuator

%% 攻击初始化
s_c_decattack = zeros(1,L);% Sensor-to-controller deception attack sequence
c_a_decattack = zeros(1,L);% Controller-to-actuator deception attack sequence
s_c_DoSattack = zeros(1,L);% Sensor-to-controller DoS attack sequence
c_a_DoSattack = zeros(1,L);% Controller-to-actuator DoS attack sequence
wx = zeros(1,L);           % Sensor-to-controller deception attack disturbance sequence
wu = zeros(1,L);           % Controller-to-actuator deception attack disturbance sequence

%% 攻击序列
for k=1:L
    s_c_decattack(k) = randsrc(1,1,[1,0;dec_rou_y,1-dec_rou_y]);
    c_a_decattack(k) = randsrc(1,1,[1,0;dec_rou_u,1-dec_rou_u]);
    
    s_c_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_y,1-DoS_rou_y]);
    c_a_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_u,1-DoS_rou_u]);

    tao_xk(1,k) = round(rand(1,1)*(tao_x-1))+1;
    tao_uk(1,k) = round(rand(1,1)*(tao_u-1))+1;

    wx(:,k) = 0.5*sin(k);
    wu(:,k) = 0.1*cos(k);
end

% load("c_a_decattack.mat")% Controller-to-actuator deception attack sequence
% load("c_a_DoSattack.mat")% Controller-to-actuator DoS attack sequence
% load("s_c_decattack.mat")% Sensor-to-controller deception attack sequence
% load("s_c_DoSattack.mat")% Sensor-to-controller DoS attack sequence
% load("tao_uk.mat")       % Delay of DoS Attack from Controller to Actuator
% load("tao_xk.mat")       % Delay of DoS Attack from Sensor to Controller

%% proposed method
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

%% Dynamic event-triggered
zeta1 = 1;                     % Momentum factors induced by dynamic events ζ
theta1 = 0.5;                  % Formula for iterating the momentum factor θ   
deta1 = 6;                     % Dynamic events activate iterative elements δ
event1 = zeros(1,L);           % Sequence of triggers
eipilon = 0.01;               % Threshold for triggering
sum_e1 = 0;                    % An error in the synthetic state has been identified

%% Set parameters for the model predictive control
Np1=8; % Predicted increment size
Nu1=2; % Control increment size

%% The process involves calculating the weighted matrix and superimposing the derived equation matrix.
At=[]; Bt=[]; temp=[];% Transformed into a matrix of derived equations concerning state variables represented in relation to control variables ut
Qt=[]; Rt=[];         % The matrix with weights following the conversion
Ft=[];Zt=zeros(size(A,1));% Nonlinearities are characterized by the consideration of incremental matrices
for i1=1:Np1
    At=[At; A^i1];
    if i1 <= Nu1
        Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i1-1)*B temp];
        temp=[A^(i1-1)*B temp];
        tempt=temp;
        Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
    else
        tempt=A*tempt;
        Bt=[Bt;tempt];
    end
    Zt = Zt+A^(i1-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
end

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk1-x1(:,k),2)^2 >= eipilon+1/deta1*zeta1
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
        ut4 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp1(:,k)+Ft*f_function(xp1(:,k),k)); 
        kexi1(k) = ut4(1);                                       % Controller output       
        xp1(:,k+1) = A*xp1(:,k)+B*kexi1(k)+f_function(xp1(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u1(:,k) = (1-c_a_DoSattack(k))*(kexi1(k)+c_a_decattack(k)*(-kexi1(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik1(tao_uk(k))+c_a_decattack(k)*(-kexik1(tao_uk(k))+wuk1(tao_uk(k))));

        event1(k) = 1;
    else
        event1(k) = 0;
        e1(:,k) = xjk1-x1(:,k);
        kexi1(k) = ut4(1); 
        xp1(:,k+1) = A*xp1(:,k)+B*kexi1(k)+f_function(xp1(:,k),k);
        u1(:,k) = u1(:,k-1);
    end

    J1(k) = x1(:,k)'*Q*x1(:,k)+u1(k)'*R*u1(k);           % Calculation of the performance index
    x1(:,k+1) = A*x1(:,k) + B*u1(k)+f_function(x1(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e1 = sum_e1+x1(:,k)'*x1(:,k);                   % Total calculation error
    zeta1 = theta1*zeta1+eipilon-norm(xjk1-x1(:,k),2)^2; % Formula for iterating the momentum factor

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

sum_t1=L/sum(event1);
sum_J1 = sum(J1);% Calculation of the performance index

%% proposed method
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
event2 = zeros(1,L);           % Sequence of triggers
sum_e2 = 0;                    % An error in the synthetic state has been identified

%% Set parameters for the model predictive control
Np2=8; % Predicted increment size
Nu2=3; % Control increment size

%% The process involves calculating the weighted matrix and superimposing the derived equation matrix.
At=[]; Bt=[]; temp=[];% Transformed into a matrix of derived equations concerning state variables represented in relation to control variables ut
Qt=[]; Rt=[];         % The matrix with weights following the conversion
Ft=[];Zt=zeros(size(A,1));% Nonlinearities are characterized by the consideration of incremental matrices
for i1=1:Np2
    At=[At; A^i1];
    if i1 <= Nu2
        Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i1-1)*B temp];
        temp=[A^(i1-1)*B temp];
        tempt=temp;
        Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
    else
        tempt=A*tempt;
        Bt=[Bt;tempt];
    end
    Zt = Zt+A^(i1-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
end

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk2-x2(:,k),2)^2 >= eipilon+1/deta2*zeta2
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
        ut4 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp2(:,k)+Ft*f_function(xp2(:,k),k)); 
        kexi2(k) = ut4(1);                                       % Controller output       
        xp2(:,k+1) = A*xp2(:,k)+B*kexi2(k)+f_function(xp2(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u2(:,k) = (1-c_a_DoSattack(k))*(kexi2(k)+c_a_decattack(k)*(-kexi2(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik2(tao_uk(k))+c_a_decattack(k)*(-kexik2(tao_uk(k))+wuk2(tao_uk(k))));

        event2(k) = 2;
    else
        event2(k) = 0;
        e2(:,k) = xjk2-x2(:,k);
        kexi2(k) = ut4(1); 
        xp2(:,k+1) = A*xp2(:,k)+B*kexi2(k)+f_function(xp2(:,k),k);
        u2(:,k) = u2(:,k-1);
    end

    J2(k) = x2(:,k)'*Q*x2(:,k)+u2(k)'*R*u2(k);           % Calculation of the performance index
    x2(:,k+1) = A*x2(:,k) + B*u2(k)+f_function(x2(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e2 = sum_e2+x2(:,k)'*x2(:,k);                   % Total calculation error
    zeta2 = theta2*zeta2+eipilon-norm(xjk2-x2(:,k),2)^2; % Formula for iterating the momentum factor

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

sum_t2=2*L/sum(event2);
sum_J2 = sum(J2);% Calculation of the performance index

%% proposed method
x3(:,1)=[-1.2;1.2];            % Initial values of system status parameters
u3(:,1)=0;                     % Initial value of the actuator parameter
xp3(:,1) = [-1.2;1.2];         % Initial value of the discriminant model
f3(:,1) = f_function(x3(:,1),1);% The original value of the nonlinear term
e3(:,1) = [0;0];               % The initial value of the trigger error
kexi3(:,1) = 0;                % The initial value of the controller output
xk3 = zeros(2,tao_x);          % The previous state of the system status
ek3 = zeros(2,tao_x);          % The previous value of the trigger error
kexik3 = zeros(1,tao_u);       % The previous value of the controller output
wxk3 = zeros(2,tao_x);         % The previous value of the disturbance from the sensor to the controller
wuk3 = zeros(1,tao_u);         % Past disturbance from the controller to the actuator
xjk3 = zeros(2,1);             % The status output value at the most recent triggering moment
J3 = zeros(1,L);               % Sequence of performance indices

%% Dynamic event-triggered
zeta3 = 1;                     % Momentum factors induced by dynamic events ζ
theta3 = 0.5;                  % Formula for iterating the momentum factor θ   
deta3 = 6;                     % Dynamic events activate iterative elements δ
event3 = zeros(1,L);           % Sequence of triggers
sum_e3 = 0;                    % An error in the synthetic state has been identified

%% Set parameters for the model predictive control
Np3=8; % Predicted increment size
Nu3=5; % Control increment size

%% The process involves calculating the weighted matrix and superimposing the derived equation matrix.
At=[]; Bt=[]; temp=[];% Transformed into a matrix of derived equations concerning state variables represented in relation to control variables ut
Qt=[]; Rt=[];         % The matrix with weights following the conversion
Ft=[];Zt=zeros(size(A,1));% Nonlinearities are characterized by the consideration of incremental matrices
for i1=1:Np3
    At=[At; A^i1];
    if i1 <= Nu3
        Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i1-1)*B temp];
        temp=[A^(i1-1)*B temp];
        tempt=temp;
        Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
    else
        tempt=A*tempt;
        Bt=[Bt;tempt];
    end
    Zt = Zt+A^(i1-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
end

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk3-x3(:,k),2)^2 >= eipilon+1/deta3*zeta3
        xjk3 = x3(:,k);                              % Update the status of the most recent trigger
        e3(:,k) = xjk3-x3(:,k);
        x_n = (1-s_c_DoSattack(k))*(x3(:,k)+e3(:,k)+s_c_decattack(k)*(-2*x3(:,k)-2*e3(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk3(:,tao_xk(k))+ek3(:,tao_xk(k))+s_c_decattack(k)*(-2*xk3(:,tao_xk(k)) ...
            -2*ek3(:,tao_xk(k))+wxk3(:,tao_xk(k)))); % The value obtained by the detection mechanism
        if norm((xp3(:,k)-x_n),2)^2 >= 0.5          % Evaluation of the detection mechanism
            xp3(:,k) = xp3(:,k);
        else
            xp3(:,k) = x_n;
        end

        %% Controller section  
        ut3 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp3(:,k)+Ft*f_function(xp3(:,k),k)); 
        kexi3(k) = ut3(1);                                       % Controller output       
        xp3(:,k+1) = A*xp3(:,k)+B*kexi3(k)+f_function(xp3(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u3(:,k) = (1-c_a_DoSattack(k))*(kexi3(k)+c_a_decattack(k)*(-kexi3(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik3(tao_uk(k))+c_a_decattack(k)*(-kexik3(tao_uk(k))+wuk3(tao_uk(k))));

        event3(k) = 3;
    else
        event3(k) = 0;
        e3(:,k) = xjk3-x3(:,k);
        kexi3(k) = ut3(1); 
        xp3(:,k+1) = A*xp3(:,k)+B*kexi3(k)+f_function(xp3(:,k),k);
        u3(:,k) = u3(:,k-1);
    end

    J3(k) = x3(:,k)'*Q*x3(:,k)+u3(k)'*R*u3(k);           % Calculation of the performance index
    x3(:,k+1) = A*x3(:,k) + B*u3(k)+f_function(x3(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e3 = sum_e3+x3(:,k)'*x3(:,k);                   % Total calculation error
    zeta3 = theta3*zeta3+eipilon-norm(xjk3-x3(:,k),2)^2; % Formula for iterating the momentum factor

    %% Data update
    for i=tao_x:-1:2
        xk3(:,i) = xk3(:,i-1);
        ek3(:,i) = ek3(:,i-1);
        wxk3(:,i) = wxk3(:,i-1);
    end
    xk3(:,1) = x3(:,k);
    ek3(:,1) = e3(:,k);
    wxk3(:,1) = wx(:,k);
    for i=tao_u:-1:2
        kexik3(i) = kexik3(i-1);
        wuk3(:,i) = wuk3(:,i-1);
    end
    kexik3(1) = kexi3(:,k);
    wuk3(:,1) = wu(:,k);
    
end

sum_t3=3*L/sum(event3);
sum_J3 = sum(J3);% Calculation of the performance index


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

%% proposed method
x4(:,1)=[-1.2;1.2];            % Initial values of system status parameters
u4(:,1)=0;                     % Initial value of the actuator parameter
xp4(:,1) = [-1.2;1.2];         % Initial value of the discriminant model
f4(:,1) = f_function(x4(:,1),1);% The original value of the nonlinear term
e4(:,1) = [0;0];               % The initial value of the trigger error
kexi4(:,1) = 0;                % The initial value of the controller output
xk4 = zeros(2,tao_x);          % The previous state of the system status
ek4 = zeros(2,tao_x);          % The previous value of the trigger error
kexik4 = zeros(1,tao_u);       % The previous value of the controller output
wxk4 = zeros(2,tao_x);         % The previous value of the disturbance from the sensor to the controller
wuk4 = zeros(1,tao_u);         % Past disturbance from the controller to the actuator
xjk4 = zeros(2,1);             % The status output value at the most recent triggering moment
J4 = zeros(1,L);               % Sequence of performance indices

%% Dynamic event-triggered
zeta4 = 1;                     % Momentum factors induced by dynamic events ζ
theta4 = 0.5;                  % Formula for iterating the momentum factor θ   
deta4 = 6;                     % Dynamic events activate iterative elements δ
event4 = zeros(1,L);           % Sequence of triggers
sum_e4 = 0;                    % An error in the synthetic state has been identified

%% Perform the operation
for k=1:L
    %%  The control action is recalculated when triggered u(k)
    if norm(xjk4-x4(:,k),2)^2 >= eipilon+1/deta4*zeta4
        xjk4 = x4(:,k);                              % Update the status of the most recent trigger
        e4(:,k) = xjk4-x4(:,k);
        x_n = (1-s_c_DoSattack(k))*(x4(:,k)+e4(:,k)+s_c_decattack(k)*(-2*x4(:,k)-2*e4(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk4(:,tao_xk(k))+ek4(:,tao_xk(k))+s_c_decattack(k)*(-2*xk4(:,tao_xk(k)) ...
            -2*ek4(:,tao_xk(k))+wxk4(:,tao_xk(k)))); % The value obtained by the detection mechanism
        if norm((xp4(:,k)-x_n),2)^2 >= 0.5          % Evaluation of the detection mechanism
            xp4(:,k) = xp4(:,k);
        else
            xp4(:,k) = x_n;
        end

        %% Controller section  
        ut4 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp4(:,k)+Ft*f_function(xp4(:,k),k)); 
        kexi4(k) = ut4(1);                                       % Controller output       
        xp4(:,k+1) = A*xp4(:,k)+B*kexi4(k)+f_function(xp4(:,k),k); % Updating a Prediction Model

        %% Component for receiving actuator input
        u4(:,k) = (1-c_a_DoSattack(k))*(kexi4(k)+c_a_decattack(k)*(-kexi4(k)+wu(k))) ... 
            +c_a_DoSattack(k)*(kexik4(tao_uk(k))+c_a_decattack(k)*(-kexik4(tao_uk(k))+wuk4(tao_uk(k))));

        event4(k) = 4;
    else
        event4(k) = 0;
        e4(:,k) = xjk4-x4(:,k);
        kexi4(k) = ut4(1); 
        xp4(:,k+1) = A*xp4(:,k)+B*kexi4(k)+f_function(xp4(:,k),k);
        u4(:,k) = u4(:,k-1);
    end

    J4(k) = x4(:,k)'*Q*x4(:,k)+u4(k)'*R*u4(k);           % Calculation of the performance index
    x4(:,k+1) = A*x4(:,k) + B*u4(k)+f_function(x4(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e4 = sum_e4+x4(:,k)'*x4(:,k);                   % Total calculation error
    zeta4 = theta4*zeta4+eipilon-norm(xjk4-x4(:,k),2)^2; % Formula for iterating the momentum factor

    %% Data update
    for i=tao_x:-1:2
        xk4(:,i) = xk4(:,i-1);
        ek4(:,i) = ek4(:,i-1);
        wxk4(:,i) = wxk4(:,i-1);
    end
    xk4(:,1) = x4(:,k);
    ek4(:,1) = e4(:,k);
    wxk4(:,1) = wx(:,k);
    for i=tao_u:-1:2
        kexik4(i) = kexik4(i-1);
        wuk4(:,i) = wuk4(:,i-1);
    end
    kexik4(1) = kexi4(:,k);
    wuk4(:,1) = wu(:,k);
    
end

sum_t4=4*L/sum(event4);
sum_J4 = sum(J4);% Calculation of the performance index

%% Simulation outcome
figure(1)
subplot(2,1,1)
i=1:L+1;
plot(i,x1(1,i),'--b',i,x2(1,i),'--g',i,x3(1,i),'--r',i,x4(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{1,k}');
% title('State output of comparison');
legend('2-step','3-step','5-step','8-step');
grid on;
axis([1 L -2 3]);
subplot(2,1,2)
i=1:L+1;
plot(i,x1(2,i),'--b',i,x2(2,i),'--g',i,x3(2,i),'--r',i,x4(2,i),'k','linewidth',2);
xlabel('k/step');
ylabel('x_{2,k}');
legend('2-step','3-step','5-step','8-step');
grid on;
axis([1 L -2 3]);

figure(2)
subplot(4,1,1)
i=1:L;
b=bar(i,s_c_decattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\alpha_{k}^x');
% title('s-c deception attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;
subplot(4,1,2)
i=1:L;
b=bar(i,s_c_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\beta_{k}^x');
% title('s-c DoS attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;
subplot(4,1,3)
i=1:L;
b=bar(i,c_a_decattack(1,i),1,'b');
set(b,'edgecolor','none')
ylabel('\alpha_{k}^u');
% title('c-a deception attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;
subplot(4,1,4)
i=1:L;
b=bar(i,c_a_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
xlabel('k/step');
ylabel('\beta_{k}^u');
% title('c-a DoS attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;

figure(3)
subplot(2,1,1)
i=1:L;
plot(i,u1(1,i),'--b',i,u2(1,i),'--g',i,u3(1,i),'--r',i,u4(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('u_k');
% title('Control law comparison');
legend('2-step','3-step','5-step','8-step');
grid on;
axis([1 L -4.5 3]);
subplot(2,1,2)
i=1:L;
sz = 50;
scatter(i,event4,sz,'s');
hold on;
scatter(i,event3,sz,'o');
hold on;
scatter(i,event2,sz,'d');
hold on;
scatter(i,event1,sz,'h');
hold on;
xlabel('k/step');
% ylabel('The triggering instants');
yticks([1 2 3 4])
yticklabels({'2-step','3-step','5-step','8-step'})
grid on;
axis([1 L 1 4]);
