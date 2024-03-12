clear; clc; close all;

L = 129; % 仿真步长

%% 设置系统系数
A=[1 0.3;0 0.8992];
B=[0;0.24];

%% 设置模型预测控制参数
Np=6;% 预测步长
Q=0.5*eye(size(A,1)); R=0.3*eye(size(B,2));% 优化目标参数，加权矩阵

%% 加权矩阵的计算过程，以及推导方程矩阵的叠加过程
At=[]; Bt=[]; temp=[];% 转化为用控制量ut表示的，关于状态量的推导方程的矩阵
Qt=[]; Rt=[];         % 转换后的加权矩阵
Ft=[];Zt=zeros(size(A,1));% 非线性想的增量矩阵
for i=1:Np
    At=[At; A^i];
    Bt=[Bt zeros(size(Bt,1),size(B,2));A^(i-1)*B temp];
    temp=[A^(i-1)*B temp];
    Zt = Zt+A^(i-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
    Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
end

%% 设置攻击参数
tao_x = 6;          % 传感器到控制器的延时
tao_u = 4;          % 控制器到执行器的延时
dec_rou_y = 0.2;    % 输出欺骗攻击概率
dec_rou_u = 0.25;   % 输入欺骗攻击概率
DoS_rou_y = 0.15;   % 输出DoS攻击概率
DoS_rou_u = 0.1;    % 输入DoS攻击概率
% tao_xk = ones(1,L); % 传感器到控制器的DoS攻击延时
% tao_uk = ones(1,L); % 控制器到执行器的DoS攻击延时

%% 攻击初始化
% s_c_decattack = zeros(1,L);% 传感器到控制器欺骗攻击序列
% c_a_decattack = zeros(1,L);% 控制器到执行器欺骗攻击序列
% s_c_DoSattack = zeros(1,L);% 传感器到控制器DoS攻击序列
% c_a_DoSattack = zeros(1,L);% 控制器到执行器DoS攻击序列
wx = zeros(1,L);           % 传感器到控制器欺骗攻击扰动序列
wu = zeros(1,L);           % 控制器到执行器欺骗攻击扰动序列

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

load("c_a_decattack.mat")% 控制器到执行器DoS攻击序列
load("c_a_DoSattack.mat")% 控制器到执行器欺骗攻击序列
load("s_c_decattack.mat")% 传感器到控制器欺骗攻击序列
load("s_c_DoSattack.mat")% 传感器到控制器DoS攻击序列
load("tao_uk.mat")       % 控制器到执行器的DoS攻击延时
load("tao_xk.mat")       % 传感器到控制器的DoS攻击延时

%% proposed method
x(:,1)=[-1.2;1.2];            % 系统状态参数初值
u(:,1)=0;                     % 执行器参数初值
xp(:,1) = [-1.2;1.2];         % 判别模型的初值
f(:,1) = f_function(x(:,1),1);% 非线性项的初值
e(:,1) = [0;0];               % 触发误差的初值
kexi(:,1) = 0;                % 控制器输出的初值
xk = zeros(2,tao_x);          % 系统状态的过去值
ek = zeros(2,tao_x);          % 触发误差的过去值
kexik = zeros(1,tao_u);       % 控制器输出的过去值
wxk = zeros(2,tao_x);         % 传感器到控制器扰动的过去值
wuk = zeros(1,tao_u);         % 控制器到执行器扰动的过去值
xjk = zeros(2,1);             % 上一触发时刻的状态输出值
J = zeros(1,L);               % 性能指标序列

%% 动态事件触发
zeta = 1;                     % 动态事件触发的动量因子ζ
theta = 0.5;                  % 动量因子迭代公式θ   
deta = 6;                     % 动态事件触发迭代因子δ
eipilon = 0.01;               % 触发阈值
event = zeros(1,L);           % 触发机制序列
sum_e = 0;                    % 综合状态误差

%% 执行运算
for k=1:L
    %%  触发时刻重新计算控制作用u(k)
    if norm(xjk-x(:,k),2)^2 >= eipilon+1/deta*zeta
        xjk = x(:,k);                              % 更新上一触发时刻的状态
        e(:,k) = xjk-x(:,k);
        x_n = (1-s_c_DoSattack(k))*(x(:,k)+e(:,k)+s_c_decattack(k)*(-2*x(:,k)-2*e(:,k)+wx(:,k))) ... 
            +s_c_DoSattack(k)*(xk(:,tao_xk(k))+ek(:,tao_xk(k))+s_c_decattack(k)*(-2*xk(:,tao_xk(k)) ...
            -2*ek(:,tao_xk(k))+wxk(:,tao_xk(k)))); % 检测机制接收值
        if norm((xp(:,k)-x_n),2)^2 >= 0.5          % 检测机制判断
            xp(:,k) = xp(:,k);
        else
            xp(:,k) = x_n;
        end

        %% 控制器部分  
        ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp(:,k)+Ft*f_function(xp(:,k),k)); 
        kexi(k) = ut(1);                                         % 控制器输出值       
        xp(:,k+1) = A*xp(:,k)+B*kexi(k)+f_function(xp(:,k),k); % 预测模型更新

        %% 执行器接收部分
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

    J(k) = x(:,k)'*Q*x(:,k)+u(k)'*R*u(k);           % 计算性能指标
    x(:,k+1) = A*x(:,k) + B*u(k)+f_function(x(:,k),k)+[0;0.036*sin(k*pi/3)]; 
    sum_e = sum_e+x(:,k)'*x(:,k);                 % 计算综合误差
    zeta = theta*zeta+eipilon-norm(xjk-x(:,k),2)^2; % 动量因子迭代公式

    %% 数据更新
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
sum_J = sum(J);% 计算性能指标

%% 仿真结果
figure(1)
subplot(2,1,1)
i=1:L+1;
plot(i,x(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{1,k}');
legend('Proposed method x_{1,k}');
grid on;
axis([0 L -2 3]);
subplot(2,1,2)
i=1:L+1;
plot(i,x(2,i),'k','linewidth',2);
xlabel('k/step');
ylabel('x_{2,k}');
legend('Proposed method x_{2,k}');
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
plot(i,u(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('u_k');
% title('Control law comparison');
legend('Proposed method u_k');
grid on;
axis([0 L -4 6]);
subplot(2,1,2)
i=1:L;
sz = 50;
scatter(i,event,sz,'d');
hold on;
xlabel('k/step');
% ylabel('The triggering instants');
yticks([1 2])
yticklabels({'Proposed method'})
grid on;
axis([0 L 1 2]);
