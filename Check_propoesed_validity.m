%% 非线性单智能体，非线性项已知
% 传感器到控制器端，控制器到执行器都存在混合攻击
%************************ 验证所提方法稳定性验证 **********************
clear; clc; close all;

L = 129;%步长设置

%% 设置系统真值
Ar=[1 0.3;0 0.8992];
Br=[0;0.24];
tic
%% 设置模型预测控制参数
Np=6;% 预测步长
Q=0.5*eye(2); R=0.3;% 优化目标参数，加权矩阵
tao_x = 6;            % 传感器到控制器的延时
tao_u = 4;            % 控制器到执行器的延时
dec_rou_y = 0.2; % 输出欺骗攻击概率
dec_rou_u = 0.25;% 输入欺骗攻击概率
DoS_rou_y = 0.15; % 输出DoS攻击概率
DoS_rou_u = 0.1;% 输入DoS攻击概率
eipilon = 0.05;   % 触发阈值
tao_xk = ones(1,L);   % 传感器到控制器的延时
tao_uk = ones(1,L);   % 传感器到控制器的延时

X=[2;2];% 状态向量的最大值
At=[]; Bt=[]; temp=[];% 转化为用控制量ut表示的，关于状态量的推导方程的矩阵
Qt=[]; Rt=[];% 转换后的加权矩阵
Xt=[];% 状态矩阵的约束项
Ut=[];% 控制矩阵的约束项
Ft=[];Zt=zeros(size(Ar,1));% 非线性想的增量矩阵

%% 攻击初始化
s_c_decattack = zeros(1,L);% 传感器到控制器欺骗攻击序列
c_a_decattack = zeros(1,L);% 控制器到执行器欺骗攻击序列
s_c_DoSattack = zeros(1,L);% 传感器到控制器DoS攻击序列
c_a_DoSattack = zeros(1,L);% 控制器到执行器DoS攻击序列

s_c_decattack1 = zeros(1,L);% 传感器到控制器欺骗攻击序列
c_a_decattack1 = zeros(1,L);% 控制器到执行器欺骗攻击序列
s_c_DoSattack1 = zeros(1,L);% 传感器到控制器DoS攻击序列
c_a_DoSattack1 = zeros(1,L);% 控制器到执行器DoS攻击序列

%% 攻击序列
for k=1:L
    s_c_decattack(k) = randsrc(1,1,[1,0;dec_rou_y,1-dec_rou_y]);
    c_a_decattack(k) = randsrc(1,1,[1,0;dec_rou_u,1-dec_rou_u]);
    
    s_c_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_y,1-DoS_rou_y]);
    c_a_DoSattack(k) = randsrc(1,1,[1,0;DoS_rou_u,1-DoS_rou_u]);

    s_c_decattack1(k) = s_c_decattack(k);
    c_a_decattack1(k) = c_a_decattack(k);
    
    s_c_DoSattack1(k) = s_c_DoSattack(k);
    c_a_DoSattack1(k) = c_a_DoSattack(k);
    
    tao_xk(1,k) = round(rand(1,1)*(tao_x-1))+1;
    tao_uk(1,k) = round(rand(1,1)*(tao_u-1))+1;

    wx(:,k) = 0.5*sin(k);
    wu(:,k) = 0.1*cos(k);
end

%% 加权矩阵的计算过程，以及推导方程矩阵的叠加过程
for i=1:Np
    At=[At; Ar^i];
    Bt=[Bt zeros(size(Bt,1),size(Br,2));Ar^(i-1)*Br temp];
    temp=[Ar^(i-1)*Br temp];
    Ut=[Ut;1.5];
    Xt=[Xt;(1-i/Np)*0.2*X];
    Zt = Zt+Ar^(i-1);
    Ft=[Ft;Zt];
    Qt=[Qt zeros(size(Qt,1),size(Q,1));zeros(size(Q,1),size(Qt,1)) Q];
    Rt=[Rt zeros(size(Rt,1),size(R,1));zeros(size(R,1),size(Rt,1)) R];
end

%% proposed method
tic
%% 历史时刻的状态和控制作用,动态事件触发
x1(:,1)=[-1.2;1.2];% 系统状态参数初值
xp1(:,1) = [-1.2;1.2];% 判别模型的初值
X1=[2;2];% 状态向量的最大值
xk1 = zeros(2,tao_x);  % 系统状态的过去值
ek1 = zeros(2,tao_x);  % 触发误差的过去值
kexik1 = zeros(1,tao_u);   % 控制器输出的过去值
wxk1 = zeros(2,tao_x);     % 传感器到控制器扰动的过去值
wuk1 = zeros(1,tao_u);     % 控制器到执行器扰动的过去值
xjk1 = zeros(2,1);   % 上一触发时刻得输出值
zeta1 = 1;            % 动态事件触发的动量因子ζ
theta1 = 0.5;         % 动量因子迭代公式θ   
deta1 = 6;            % 动态事件触发迭代因子δ
sum_e1 = 0;

%% 执行运算
for k=1:L
    f1(:,k)=[0;-0.216*exp(-x1(1,k))*x1(1,k)-0.048*cos(k*pi/5)*x1(2,k)];
    e1(:,k) = xjk1-x1(:,k);
    %%  触发时刻重新计算控制作用u(k)
%     if norm(x1(:,k)-xjk1,2)^2 >= 0.5*norm(x1(:,k),2)^2
    if norm(x1(:,k)-xjk1,2)^2 >= eipilon+1/deta1*zeta1
        xjk1 = x1(:,k);% 更新上一触发时刻的状态
        e1(:,k) = xjk1-x1(:,k);
        x_n = (1-s_c_DoSattack1(k))*(x1(:,k)+e1(:,k)+s_c_decattack1(k)*(-2*x1(:,k)-2*e1(:,k)+wx(:,k))) ... 
            +s_c_DoSattack1(k)*(xk1(:,tao_xk(k))+ek1(:,tao_xk(k))+s_c_decattack1(k)*(-2*xk1(:,tao_xk(k)) ...
            -2*ek1(:,tao_xk(k))+wxk1(:,tao_xk(k))));% 存在欺骗攻击
        if norm((xp1(:,k)-x_n),2)^2 > 0.1
            xp1(:,k+1) = xp1(:,k);
            detector1(k) = 1; % 有发生攻击
        else
            xp1(:,k+1) = x_n;
            detector1(k) = 0; % 有发生攻击
        end

        %% 控制器部分
        f_p1(:,k)=[0;-0.216*exp(-xp1(1,k+1))*xp1(1,k+1)-0.048*cos(k*pi/5)*xp1(2,k+1)];   
        ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp1(:,k+1)+Ft*f_p1(:,k));
        K1_m = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*At;
        K2_m = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*Ft;        
        K1 = K1_m(1,:);  
        K2 = K2_m(1,:);
        
        xp1(:,k+1) = Ar*xp1(:,k+1)+Br*ut(1)+f_p1(:,k);
        kexi1(k) = ut(1); % 检测攻击后第一次的传输不会被攻击

        %% 执行器部分
        u1(:,k) = (1-c_a_DoSattack1(k))*(kexi1(k)+c_a_decattack1(k)*(-kexi1(k)+wu(k))) ... 
            +c_a_DoSattack1(k)*(kexik1(tao_uk(k))+c_a_decattack1(k)*(-kexik1(tao_uk(k))+wuk1(tao_uk(k))));
        event1(k) = 1;
    else
        event1(k) = 0;
        e1(:,k) = xjk1-x1(:,k);
        kexi1(k) = ut(1);
        f_p1(:,k)=[0;-0.216*exp(-xp1(1,k))*xp1(1,k)-0.048*cos(k*pi/5)*xp1(2,k)];  
        xp1(:,k+1) = Ar*xp1(:,k)+Br*ut(1)+f_p1(:,k);
        u1(:,k) = u1(:,k-1);
    end
    
    if k ==1
        beita_x = dec_rou_y*(1-dec_rou_y); % 传感器到控制器的欺骗攻击方差
        beita_u = dec_rou_u*(1-dec_rou_u); % 控制器到执行器的欺骗攻击方差
        afa_x = DoS_rou_y*(1-DoS_rou_y);   % 传感器到控制器的DoS攻击方差
        afa_u = DoS_rou_u*(1-DoS_rou_u);   % 控制器到执行器的DoS攻击方差

        A=[Ar+(1-DoS_rou_u)*(1-dec_rou_u)*Br*K1 (1-DoS_rou_u)*(1-dec_rou_u)*Br*K1;(1-(1-DoS_rou_u)*(1-dec_rou_u))*Br*K1 Ar+(1-(1-DoS_rou_u)*(1-dec_rou_u))*Br*K1];
        A1=[-(1-dec_rou_u)*Br*K1 -(1-dec_rou_u)*Br*K1;(1-dec_rou_u)*Br*K1 (1-dec_rou_u)*Br*K1];
        A2=[-(1-DoS_rou_u)*Br*K1 -(1-DoS_rou_u)*Br*K1;(1-DoS_rou_u)*Br*K1 (1-DoS_rou_u)*Br*K1];
        A3=[Br*K1 Br*K1;-Br*K1 -Br*K1];
        A4 = [eye(2) zeros(2,2)];
        A5 = [eye(2) eye(2,2)];
        A6 = [zeros(2,2) eye(2,2)];
        A_b=[(1-(1-DoS_rou_y)*(1-2*dec_rou_y))*eye(2) eye(2)];
        A_b1=[((1-2*dec_rou_y))*eye(2) zeros(2,2)];
        A_b2=[(2*(1-DoS_rou_y))*eye(2) zeros(2,2)];
        A_b3=[-2*eye(2) zeros(2,2)];

        B=[eye(2) (1-DoS_rou_u)*(1-dec_rou_u)*Br*K2;-eye(2) eye(2)+(1-(1-DoS_rou_u)*(1-dec_rou_u))*Br*K2];
        B1=[zeros(2,2) -(1-dec_rou_u)*Br*K2;zeros(2,2) (1-dec_rou_u)*Br*K2];
        B2=[zeros(2,2) -(1-DoS_rou_u)*Br*K2;zeros(2,2) (1-DoS_rou_u)*Br*K2];
        B3=[zeros(2,2) Br*K2;zeros(2,2) -Br*K2];
        B_b=-(1-DoS_rou_y)*(1-2*dec_rou_y);
        B_b1=(1-2*dec_rou_y);
        B_b2=2*(1-DoS_rou_y);
        B_b3=-2;

        C=[(DoS_rou_u)*(1-dec_rou_u)*Br*K1 (DoS_rou_u)*(1-dec_rou_u)*Br*K1;-(DoS_rou_u)*(1-dec_rou_u)*Br*K1 -(DoS_rou_u)*(1-dec_rou_u)*Br*K1];
        C1=[(1-dec_rou_u)*Br*K1 (1-dec_rou_u)*Br*K1;-(1-dec_rou_u)*Br*K1 -(1-dec_rou_u)*Br*K1];
        C2=[-(DoS_rou_u)*Br*K1 -(DoS_rou_u)*Br*K1;(DoS_rou_u)*Br*K1 (DoS_rou_u)*Br*K1];
        C3=[-Br*K1 -Br*K1;Br*K1 Br*K1];
        C_b=[DoS_rou_y*(2*dec_rou_y-1)*eye(2) zeros(2,2)];
        C_b1=[-(1-2*dec_rou_y)*eye(2) zeros(2,2)];
        C_b2=[2*(DoS_rou_y)*eye(2) zeros(2,2)];
        C_b3=[2*eye(2) zeros(2,2)];

        D=[zeros(2,2) (DoS_rou_u)*(1-dec_rou_u)*Br*K2;zeros(2,2) -(DoS_rou_u)*(1-dec_rou_u)*Br*K2];
        D1=[zeros(2,2) (1-dec_rou_u)*Br*K2;zeros(2,2) -(1-dec_rou_u)*Br*K2];
        D2=[zeros(2,2) -(DoS_rou_u)*Br*K2;zeros(2,2) (DoS_rou_u)*Br*K2];
        D3=[zeros(2,2) -Br*K2;zeros(2,2) Br*K2];
        D_b=DoS_rou_y*(2*dec_rou_y-1);
        D_b1=-(1-2*dec_rou_y);
        D_b2=2*(DoS_rou_y);
        D_b3=2;

        F=[eye(2) (1-DoS_rou_u)*(dec_rou_u)*Br;-eye(2) -(1-DoS_rou_u)*(dec_rou_u)*Br];
        F1=[zeros(2,2) -(dec_rou_u)*Br;zeros(2,2) (dec_rou_u)*Br];
        F2=[zeros(2,2) (1-DoS_rou_u)*Br;zeros(2,2) -(1-DoS_rou_u)*Br];
        F3=[zeros(2,2) -Br;zeros(2,2) Br];
        F_b=-(1-DoS_rou_y)*(dec_rou_y);
        F_b1=(dec_rou_y);
        F_b2=-(1-DoS_rou_y);
        F_b3=1;

        G=[zeros(2,2) (DoS_rou_u)*(dec_rou_y)*Br;zeros(2,2) -(DoS_rou_u)*(dec_rou_y)*Br];
        G1=[zeros(2,2) (dec_rou_y)*Br;zeros(2,2) -(dec_rou_y)*Br];
        G2=[zeros(2,2) (DoS_rou_u)*Br;zeros(2,2) -(DoS_rou_u)*Br];
        G3=[zeros(2,2) Br;zeros(2,2) -Br];
        G_b=-(DoS_rou_y)*(dec_rou_y);
        G_b1=-(dec_rou_y);
        G_b2=-(1-DoS_rou_y);
        G_b3=-1;

        setlmis([])
        P =lmivar(1,[4 1]);
        Q1=lmivar(1,[4 1]);
        Q2=lmivar(1,[4 1]);

        landat = 0.1;
        sigma1 = 0.05;
        sigma2 = 0.1;
        eipilong = 0.5;

        afa=lmivar(1,[1 1]);% 状态x收敛的范围α
        gama=lmivar(1,[1 1]); % 扰动w收敛的范围γ
        kapa=lmivar(1,[1 1]);% 状态x收敛的范围κ
        theta=lmivar(1,[1 1]); % 扰动w收敛的范围θ
        pai1=lmivar(1,[1 1]); % 动态事件触发约束项
        pai2=lmivar(1,[1 1]); % 检测机制约束项
        miu1=lmivar(1,[1 1]); % 非线性约束项
        miu2=lmivar(1,[1 1]); % 非线性约束项
        miu3=lmivar(1,[1 1]); % 非线性约束项
        miu4=lmivar(1,[1 1]); % 非线性约束项

        lmiterm([1 1 1 P],-1,1)
        lmiterm([1 1 1 Q1],(1+tao_u-1),1)
        lmiterm([1 1 1 Q2],(1+tao_x-1),1)
        lmiterm([1 1 1 miu1],landat*A4',A4)
        lmiterm([1 1 1 miu2],landat*A5',A5)
        lmiterm([1 1 1 pai2],(eipilong-1)*A_b',A_b)
        lmiterm([1 1 1 pai2],(eipilong-1)*afa_x*A_b1',A_b1)
        lmiterm([1 1 1 pai2],(eipilong-1)*beita_x*A_b2',A_b2)
        lmiterm([1 1 1 pai2],(eipilong-1)*afa_x*beita_x*A_b3',A_b3)
        lmiterm([1 1 1 afa],eye(4),1)

        lmiterm([1 1 3 pai2],(eipilong-1)*A_b',B_b)
        lmiterm([1 1 3 pai2],(eipilong-1)*afa_x*A_b1',B_b1)
        lmiterm([1 1 3 pai2],(eipilong-1)*beita_x*A_b2',B_b2)
        lmiterm([1 1 3 pai2],(eipilong-1)*afa_x*beita_x*A_b3',B_b3)

        lmiterm([1 1 6 pai2],(eipilong-1)*A_b',C_b)
        lmiterm([1 1 6 pai2],(eipilong-1)*afa_x*A_b1',C_b1)
        lmiterm([1 1 6 pai2],(eipilong-1)*beita_x*A_b2',C_b2)
        lmiterm([1 1 6 pai2],(eipilong-1)*afa_x*beita_x*A_b3',C_b3)

        lmiterm([1 1 7 pai2],(eipilong-1)*A_b',D_b)
        lmiterm([1 1 7 pai2],(eipilong-1)*afa_x*A_b1',D_b1)
        lmiterm([1 1 7 pai2],(eipilong-1)*beita_x*A_b2',D_b2)
        lmiterm([1 1 7 pai2],(eipilong-1)*afa_x*beita_x*A_b3',D_b3)

        lmiterm([1 2 2 miu1],-A4',A4)   
        lmiterm([1 2 2 miu2],-A6',A6)   

        lmiterm([1 3 3 pai1],-eye(2),1)        
        lmiterm([1 3 3 pai2],(eipilong-1)*B_b',B_b)
        lmiterm([1 3 3 pai2],(eipilong-1)*afa_x*B_b1',B_b1)
        lmiterm([1 3 3 pai2],(eipilong-1)*beita_x*B_b2',B_b2)
        lmiterm([1 3 3 pai2],(eipilong-1)*afa_x*beita_x*B_b3',B_b3)

        lmiterm([1 3 6 pai2],(eipilong-1)*B_b',C_b)
        lmiterm([1 3 6 pai2],(eipilong-1)*afa_x*B_b1',C_b1)
        lmiterm([1 3 6 pai2],(eipilong-1)*beita_x*B_b2',C_b2)
        lmiterm([1 3 6 pai2],(eipilong-1)*afa_x*beita_x*B_b3',C_b3)

        lmiterm([1 3 7 pai2],(eipilong-1)*B_b',D_b)
        lmiterm([1 3 7 pai2],(eipilong-1)*afa_x*B_b1',D_b1)
        lmiterm([1 3 7 pai2],(eipilong-1)*beita_x*B_b2',D_b2)
        lmiterm([1 3 7 pai2],(eipilong-1)*afa_x*beita_x*B_b3',D_b3)

        lmiterm([1 4 4 Q2],-1,1)      
        lmiterm([1 4 4 miu3],landat*A4',A4)   
        lmiterm([1 4 4 miu4],landat*A5',A5)  

        lmiterm([1 5 5 miu3],-A4',A4)
        lmiterm([1 5 5 miu4],-A6',A6)

        lmiterm([1 6 6 Q1],-1,1)     
        lmiterm([1 6 6 pai2],(eipilong-1)*C_b',C_b*eye(4))
        lmiterm([1 6 6 pai2],(eipilong-1)*afa_x*C_b1',C_b1*eye(4))
        lmiterm([1 6 6 pai2],(eipilong-1)*beita_x*C_b2',C_b2*eye(4))
        lmiterm([1 6 6 pai2],(eipilong-1)*afa_x*beita_x*C_b3',C_b3*eye(4))     

        lmiterm([1 6 7 pai2],(eipilong-1)*C_b',D_b)
        lmiterm([1 6 7 pai2],(eipilong-1)*afa_x*C_b1',D_b1)
        lmiterm([1 6 7 pai2],(eipilong-1)*beita_x*C_b2',D_b2)
        lmiterm([1 6 7 pai2],(eipilong-1)*afa_x*beita_x*C_b3',D_b3)   

        lmiterm([1 7 7 pai2],(eipilong-1)*D_b',D_b)
        lmiterm([1 7 7 pai2],(eipilong-1)*afa_x*D_b1',D_b1)
        lmiterm([1 7 7 pai2],(eipilong-1)*beita_x*D_b2',D_b2)
        lmiterm([1 7 7 pai2],(eipilong-1)*afa_x*beita_x*D_b3',D_b3)  

        lmiterm([1 8 1 0],sqrt(1+eipilong)*A)
        lmiterm([1 8 2 0],sqrt(1+eipilong)*B)
        lmiterm([1 8 4 0],sqrt(1+eipilong)*C)
        lmiterm([1 8 5 0],sqrt(1+eipilong)*D) 

        lmiterm([1 9 1 0],sqrt(1+eipilong)*sqrt(afa_u)*A1)
        lmiterm([1 9 2 0],sqrt(1+eipilong)*sqrt(afa_u)*B1)
        lmiterm([1 9 4 0],sqrt(1+eipilong)*sqrt(afa_u)*C1)
        lmiterm([1 9 5 0],sqrt(1+eipilong)*sqrt(afa_u)*D1)

        lmiterm([1 10 1 0],sqrt(1+eipilong)*sqrt(beita_u)*A2)
        lmiterm([1 10 2 0],sqrt(1+eipilong)*sqrt(beita_u)*A2)
        lmiterm([1 10 4 0],sqrt(1+eipilong)*sqrt(beita_u)*A2)
        lmiterm([1 10 5 0],sqrt(1+eipilong)*sqrt(beita_u)*A2)

        lmiterm([1 11 1 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*A3)
        lmiterm([1 11 2 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*B3)
        lmiterm([1 11 4 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*C3)
        lmiterm([1 11 5 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*D3)

        lmiterm([1 8 8 inv(P)],-1,1)
        lmiterm([1 9 9 inv(P)],-1,1)
        lmiterm([1 10 10 inv(P)],-1,1)
        lmiterm([1 11 11 inv(P)],-1,1)

        lmiterm([2 1 1 gama],-eye(3),1)
        lmiterm([2 2 2 gama],-eye(3),1) 

        lmiterm([2 3 1 0],sqrt(1+1/eipilong)*F)
        lmiterm([2 3 2 0],sqrt(1+1/eipilong)*G)

        lmiterm([2 4 1 0],sqrt(1+1/eipilong)*sqrt(afa_u)*F1)
        lmiterm([2 4 2 0],sqrt(1+1/eipilong)*sqrt(afa_u)*G1)

        lmiterm([2 5 1 0],sqrt(1+1/eipilong)*sqrt(beita_u)*F2)
        lmiterm([2 5 2 0],sqrt(1+1/eipilong)*sqrt(beita_u)*G2)

        lmiterm([2 6 1 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*F3)
        lmiterm([2 6 2 0],sqrt(1+eipilong)*sqrt(afa_u*beita_u)*G3)

        lmiterm([2 3 3 inv(P)],-1,1)
        lmiterm([2 4 4 inv(P)],-1,1)
        lmiterm([2 5 5 inv(P)],-1,1)
        lmiterm([2 6 6 inv(P)],-1,1)

        lmiterm([3 1 1 kapa],-1,1)
        lmiterm([3 2 2 kapa],-1,1) 

        lmiterm([3 3 1 sqrt(pai2)],sqrt(1+1/eipilong)*F_b,1)
        lmiterm([3 3 2 sqrt(pai2)],sqrt(1+1/eipilong)*G_b,1)

        lmiterm([3 4 1 sqrt(pai2)],sqrt(1+1/eipilong)*sqrt(afa_x)*F_b1,1)
        lmiterm([3 4 2 sqrt(pai2)],sqrt(1+1/eipilong)*sqrt(afa_x)*G_b1,1)

        lmiterm([3 5 1 sqrt(pai2)],sqrt(1+1/eipilong)*sqrt(beita_x)*F_b2,1)
        lmiterm([3 5 2 sqrt(pai2)],sqrt(1+1/eipilong)*sqrt(beita_x)*G_b2,1)

        lmiterm([3 6 1 sqrt(pai2)],sqrt(1+eipilong)*sqrt(afa_x*beita_x)*F_b3,1)
        lmiterm([3 6 2 sqrt(pai2)],sqrt(1+eipilong)*sqrt(afa_x*beita_x)*G_b3,1)

        lmiterm([3 3 3 0],-1)
        lmiterm([3 4 4 0],-1)
        lmiterm([3 5 5 0],-1)
        lmiterm([3 6 6 0],-1)    

        lmiterm([4 1 1 theta],1,1)
        lmiterm([4 1 1 pai1],1/6,1)
        lmiterm([4 1 1 0],-0.5/6)

        lmiterm([5 1 1 P],-1,1)
        lmiterm([6 1 1 Q1],-1,1)
        lmiterm([7 1 1 Q2],-1,1)
        lmiterm([8 1 1 afa],-1,1)
        lmiterm([9 1 1 gama],-1,1)
        lmiterm([10 1 1 kapa],-1,1)
        lmiterm([11 1 1 theta],-1,1)
        lmiterm([12 1 1 pai1],-1,1)
        lmiterm([13 1 1 pai2],-1,1)
        lmiterm([14 1 1 miu1],-1,1)
        lmiterm([15 1 1 miu2],-1,1)
        lmiterm([16 1 1 miu3],-1,1)
        lmiterm([17 1 1 miu4],-1,1)

        lmis=getlmis;
        % s=eig(A);
        [tmin,xfeas]=feasp(lmis);

        P=dec2mat(lmis,xfeas,P);
        Q1=dec2mat(lmis,xfeas,Q1);
        Q2=dec2mat(lmis,xfeas,Q2);
        afa=dec2mat(lmis,xfeas,afa);
        gama=dec2mat(lmis,xfeas,gama);
        kapa=dec2mat(lmis,xfeas,kapa);
        theta=dec2mat(lmis,xfeas,theta);
        pai1=dec2mat(lmis,xfeas,pai1);
        pai2=dec2mat(lmis,xfeas,pai2);
        miu1=dec2mat(lmis,xfeas,miu1);
        miu2=dec2mat(lmis,xfeas,miu2);
        miu3=dec2mat(lmis,xfeas,miu3);
        miu4=dec2mat(lmis,xfeas,miu4);
    end

    J1(k) = x1(:,k)'*Q*x1(:,k)+u1(k)'*R*u1(k);
    x1(:, k+1) = Ar*x1(:, k) + Br*u1(k)+f1(:,k)+[0;0.036*sin(k*pi/3)]; 
    sum_e1 = sum_e1+x1(:, k)'*x1(:, k);% 计算综合误差
    zeta1 = theta1*zeta1+eipilon-norm(x1(:,k)-xjk1,2)^2; % 动量因子迭代公式

    %% 数据更新
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
toc
sum_t1=L/sum(event1);
% sum_t1=sum(event1);
sum_J1 = sum(J1);% 计算性能指标

%% 仿真结果
figure(1)
subplot(2,1,1)
i=1:L+1;
plot(i,x1(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{1,k}');
% title('State output of comparison');
legend('Proposed method x_{1,k}');
grid on;
axis([0 L -2 3]);
subplot(2,1,2)
i=1:L+1;
plot(i,x1(2,i),'k','linewidth',2);
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
plot(i,u1(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('u_k');
% title('Control law comparison');
legend('Proposed method u_k');
grid on;
axis([0 L -3 3]);
subplot(2,1,2)
i=1:L;
sz = 50;
scatter(i,event1,sz,'d');
hold on;
xlabel('k/step');
% ylabel('The triggering instants');
yticks([1 2 3])
yticklabels({'Proposed method'})
grid on;
axis([0 L 1 3]);
