%% 非线性单智能体，非线性项已知
% 传感器到控制器端，控制器到执行器都存在混合攻击
% ************************ 对比动态事件触发下不同控制方法 **********************

clear; clc; close all;

L = 69;%步长设置

%% 设置系统真值
Ar=[1 0.3;0 0.8992];
Br=[0;0.24];
success = 0;
for i=1:20 
    %% 设置模型预测控制参数
    Np=6;% 预测步长
    Q=0.5*eye(2); R=0.3;% 优化目标参数，加权矩阵
    tao_x = 6;            % 传感器到控制器的延时
    tao_u = 4;            % 控制器到执行器的延时
    dec_rou_y = 0.2; % 输出欺骗攻击概率
    dec_rou_u = 0.25;% 输入欺骗攻击概率
    DoS_rou_y = 0.15; % 输出DoS攻击概率
    DoS_rou_u = 0.1;% 输入DoS攻击概率
    eipilon = 0.01;   % 触发阈值
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
    
    s_c_decattack4 = zeros(1,L);% 传感器到控制器欺骗攻击序列
    c_a_decattack4 = zeros(1,L);% 控制器到执行器欺骗攻击序列
    s_c_DoSattack4 = zeros(1,L);% 传感器到控制器DoS攻击序列
    c_a_DoSattack4 = zeros(1,L);% 控制器到执行器DoS攻击序列
    
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
    
        s_c_decattack4(k) = s_c_decattack(k);
        c_a_decattack4(k) = c_a_decattack(k);
        
        s_c_DoSattack4(k) = s_c_DoSattack(k);
        c_a_DoSattack4(k) = c_a_DoSattack(k);
    
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
    
    %% DET-反馈控制
    x4(:,1)=[-1.2;1.2];% 系统状态参数初值
    
    %% 历史时刻的状态和控制作用
    xk4 = zeros(2,tao_x);  % 系统状态的过去值
    ek4 = zeros(2,tao_x);  % 触发误差的过去值
    kexik4 = zeros(1,tao_u);   % 控制器输出的过去值
    wxk4 = zeros(2,tao_x);     % 传感器到控制器扰动的过去值
    wuk4 = zeros(1,tao_u);     % 控制器到执行器扰动的过去值
    xjk4 = zeros(2,1);% 上一触发时刻得输出值
    zeta4 = 1;            % 动态事件触发的动量因子ζ
    theta4 = 0.5;         % 动量因子迭代公式θ   
    deta4 = 6;            % 动态事件触发迭代因子δ
    landat = 0.1;    % 非线性约束项
    sum_e4 = 0;
    
    %% 执行运算
    for k=1:L
        f4(:,k)=[0;-0.216*exp(-x4(1,k))*x4(1,k)-0.048*cos(k*pi/5)*x4(2,k)];
        e4(:,k) = xjk4-x4(:,k);
        %%  触发时刻重新计算控制作用u(k)
        if norm(x4(:,k)-xjk4,2)^2 >= 0.01*norm(x4(:,k),2)^2+1/deta4*zeta4
            xjk4 = x4(:,k);% 更新上一触发时刻的状态
            e4(:,k) = xjk4-x4(:,k);
            x_n = (1-s_c_DoSattack4(k))*(x4(:,k)+e4(:,k)+s_c_decattack4(k)*(-2*x4(:,k)-2*e4(:,k)+wx(:,k))) ... 
                +s_c_DoSattack4(k)*(xk4(:,tao_xk(k))+ek4(:,tao_xk(k))+s_c_decattack4(k)*(-2*xk4(:,tao_xk(k)) ...
                -2*ek4(:,tao_xk(k))+wxk4(:,tao_xk(k))));% 存在欺骗攻击
            %% 求解LMI,验证是否有解
            if k<=1
                beita_x = dec_rou_y*(1-dec_rou_y); % 传感器到控制器的欺骗攻击方差
                beita_u = dec_rou_u*(1-dec_rou_u); % 控制器到执行器的欺骗攻击方差
                afa_x = DoS_rou_y*(1-DoS_rou_y);   % 传感器到控制器的DoS攻击方差
                afa_u = DoS_rou_u*(1-DoS_rou_u);   % 控制器到执行器的DoS攻击方差
        
                A=(1-DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                A1=(1-DoS_rou_u)*(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                A2=2*(1-DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                A3=(1-dec_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                A4=(1-DoS_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                A5=2*(1-DoS_rou_u)*(1-dec_rou_u)*Br;
                A6=(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                A7=(1-DoS_rou_u)*(1-2*dec_rou_y)*Br;
                A8=2*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                A9=2*(1-DoS_rou_u)*(1-DoS_rou_y)*Br;
                A10=(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                A11=2*(1-dec_rou_u)*Br;
                A12=2*(1-DoS_rou_u)*Br;
                A13=(1-2*dec_rou_y)*Br;
                A14=2*(1-DoS_rou_y)*Br;
                A15=2*Br;
        
                B=(1-DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                B1=-(1-DoS_rou_u)*(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                B2=2*(1-DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*Br;
                B3=(1-dec_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                B4=(1-DoS_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                B5=-2*(1-DoS_rou_u)*(1-dec_rou_u)*Br;
                B6=-(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                B7=-(1-DoS_rou_u)*(1-2*dec_rou_y)*Br;
                B8=2*(1-dec_rou_u)*(DoS_rou_y)*Br;
                B9=2*(1-DoS_rou_u)*(DoS_rou_y)*Br;
                B10=(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                B11=-2*(1-dec_rou_u)*Br;
                B12=-2*(1-DoS_rou_u)*Br;
                B13=-(1-2*dec_rou_y)*Br;
                B14=2*(DoS_rou_y)*Br;
                B15=-2*Br;
                
                C=(DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                C1=(DoS_rou_u)*(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                C2=2*(DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                C3=-(1-dec_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                C4=(DoS_rou_u)*(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                C5=2*(DoS_rou_u)*(1-dec_rou_u)*Br;
                C6=-(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                C7=(DoS_rou_u)*(1-2*dec_rou_y)*Br;
                C8=-2*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                C9=2*(DoS_rou_u)*(1-DoS_rou_y)*Br;
                C10=-(1-DoS_rou_y)*(1-2*dec_rou_y)*Br;
                C11=-2*(1-dec_rou_u)*Br;
                C12=2*(DoS_rou_u)*Br;
                C13=-(1-2*dec_rou_y)*Br;
                C14=-2*(1-DoS_rou_y)*Br;
                C15=-2*Br;
        
                D=(DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                D1=-(DoS_rou_u)*(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                D2=2*(DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*Br;
                D3=-(1-dec_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                D4=(DoS_rou_u)*(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                D5=-2*(DoS_rou_u)*(1-dec_rou_u)*Br;
                D6=(1-dec_rou_u)*(1-2*dec_rou_y)*Br;
                D7=-(DoS_rou_u)*(1-2*dec_rou_y)*Br;
                D8=-2*(1-dec_rou_u)*(DoS_rou_y)*Br;
                D9=2*(DoS_rou_u)*(DoS_rou_y)*Br;
                D10=-(DoS_rou_y)*(1-2*dec_rou_y)*Br;
                D11=2*(1-dec_rou_u)*Br;
                D12=-2*(DoS_rou_u)*Br;
                D13=(1-2*dec_rou_y)*Br;
                D14=-2*(DoS_rou_y)*Br;
                D15=2*Br;
        
                E=(1-DoS_rou_u)*dec_rou_u*Br;
                E1=0;E2=0;E5=0;E6=0;E7=0;E8=0;E9=0;E11=0;E12=0;E13=0;E14=0;E15=0;
                E3=dec_rou_u*Br;
                E4=-(1-DoS_rou_u)*Br;
                E10=-Br;
        
                F=(1-DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                F1=-(1-DoS_rou_u)*(1-dec_rou_u)*(dec_rou_y)*Br;
                F2=-(1-DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*Br;
                F3=(1-dec_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                F4=(1-DoS_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                F5=(1-DoS_rou_u)*(1-dec_rou_u)*Br;
                F6=-(1-dec_rou_u)*(dec_rou_y)*Br;
                F7=-(1-DoS_rou_u)*(dec_rou_y)*Br;
                F8=-(1-dec_rou_u)*(DoS_rou_y)*Br;
                F9=-(1-DoS_rou_u)*(DoS_rou_y)*Br;
                F10=(DoS_rou_y)*(dec_rou_y)*Br;
                F11=(1-dec_rou_u)*Br;
                F12=(1-DoS_rou_u)*Br;
                F13=-(dec_rou_y)*Br;
                F14=-(DoS_rou_y)*Br;
                F15=Br;
        
                G=(DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                G1=(DoS_rou_u)*(1-dec_rou_u)*(dec_rou_y)*Br;
                G2=-(DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                G3=-(1-dec_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                G4=(DoS_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                G5=-(DoS_rou_u)*(1-dec_rou_u)*Br;
                G6=-(1-dec_rou_u)*(dec_rou_y)*Br;
                G7=(DoS_rou_u)*(dec_rou_y)*Br;
                G8=(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                G9=-(DoS_rou_u)*(1-DoS_rou_y)*Br;
                G10=-(1-DoS_rou_y)*(dec_rou_y)*Br;
                G11=(1-dec_rou_u)*Br;
                G12=-(DoS_rou_u)*Br;
                G13=-(dec_rou_y)*Br;
                G14=(1-DoS_rou_y)*Br;
                G15=Br;
        
                H=(DoS_rou_u)*dec_rou_u*Br;
                H1=0;H2=0;H5=0;H6=0;H7=0;H8=0;H9=0;H11=0;H12=0;H13=0;H14=0;H15=0;
                H3=-dec_rou_u*Br;
                H4=-DoS_rou_u*Br;
                H10=Br;
        
                Lt=(DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                L1=-(DoS_rou_u)*(1-dec_rou_u)*(dec_rou_y)*Br;
                L2=-(DoS_rou_u)*(1-dec_rou_u)*(DoS_rou_y)*Br;
                L3=-(1-dec_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                L4=(DoS_rou_u)*(DoS_rou_y)*(dec_rou_y)*Br;
                L5=(DoS_rou_u)*(1-dec_rou_u)*Br;
                L6=(1-dec_rou_u)*(dec_rou_y)*Br;
                L7=-(DoS_rou_u)*(dec_rou_y)*Br;
                L8=(1-dec_rou_u)*(DoS_rou_y)*Br;
                L9=-(DoS_rou_u)*(DoS_rou_y)*Br;
                L10=-(DoS_rou_y)*(dec_rou_y)*Br;
                L11=-(1-dec_rou_u)*Br;
                L12=(DoS_rou_u)*Br;
                L13=(dec_rou_y)*Br;
                L14=(DoS_rou_y)*Br;
                L15=-Br;
        
                S=(1-DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                S1=(1-DoS_rou_u)*(1-dec_rou_u)*(dec_rou_y)*Br;
                S2=-(1-DoS_rou_u)*(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                S3=(1-dec_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                S4=(1-DoS_rou_u)*(1-DoS_rou_y)*(dec_rou_y)*Br;
                S5=-(1-DoS_rou_u)*(1-dec_rou_u)*Br;
                S6=(1-dec_rou_u)*(dec_rou_y)*Br;
                S7=(1-DoS_rou_u)*(dec_rou_y)*Br;
                S8=-(1-dec_rou_u)*(1-DoS_rou_y)*Br;
                S9=-(1-DoS_rou_u)*(1-DoS_rou_y)*Br;
                S10=(1-DoS_rou_y)*(dec_rou_y)*Br;
                S11=-(1-dec_rou_u)*Br;
                S12=-(1-DoS_rou_u)*Br;
                S13=(dec_rou_y)*Br;
                S14=-(1-DoS_rou_y)*Br;
                S15=-Br;
               
                
                setlmis([])
                P =lmivar(1,[2 1]);
                Q1=lmivar(1,[2 1]);
                Q2=lmivar(1,[2 1]);
                Q3=lmivar(1,[2 1]);
                K =lmivar(2,[1 2]);
        
                miu=lmivar(1,[1 1]);  % 非线性约束
                beita=lmivar(1,[1 1]);% 状态x收敛的范围β
                gama=lmivar(1,[1 1]); % 扰动w收敛的范围γ
                
                lmiterm([1 1 1 P],-1,1)
                lmiterm([1 1 1 Q1],tao_x,1)
                lmiterm([1 1 1 Q2],tao_u,1)
                lmiterm([1 1 1 Q3],tao_x+tao_u-1,1)
                lmiterm([1 1 1 miu],landat*eye(2),1)
                lmiterm([1 1 1 beita],eye(2),1)
        
                lmiterm([1 2 2 Q1],-1,1)   
                lmiterm([1 3 3 Q2],-1,1)   
                lmiterm([1 4 4 Q3],-1,1)        
                lmiterm([1 5 5 miu],-eye(2),1)
        
                lmiterm([1 6 1 0],sqrt(1+0.5)*Ar)
                lmiterm([1 6 1 K],sqrt(1+0.5)*A,1)
                lmiterm([1 6 2 K],sqrt(1+0.5)*B,1)
                lmiterm([1 6 3 K],sqrt(1+0.5)*C,1)
                lmiterm([1 6 4 K],sqrt(1+0.5)*D,1)
                lmiterm([1 6 5 0],sqrt(1+0.5)*eye(2))
        
                lmiterm([1 7 1 K],sqrt(1+0.5)*sqrt(afa_x)*A1,1)
                lmiterm([1 7 2 K],sqrt(1+0.5)*sqrt(afa_x)*B1,1)
                lmiterm([1 7 3 K],sqrt(1+0.5)*sqrt(afa_x)*C1,1)
                lmiterm([1 7 4 K],sqrt(1+0.5)*sqrt(afa_x)*D1,1)
        
                lmiterm([1 8 1 K],sqrt(1+0.5)*sqrt(beita_x)*A2,1)
                lmiterm([1 8 2 K],sqrt(1+0.5)*sqrt(beita_x)*B2,1)
                lmiterm([1 8 3 K],sqrt(1+0.5)*sqrt(beita_x)*C2,1)
                lmiterm([1 8 4 K],sqrt(1+0.5)*sqrt(beita_x)*D2,1)
        
                lmiterm([1 9 1 K],sqrt(1+0.5)*sqrt(afa_u)*A3,1)
                lmiterm([1 9 2 K],sqrt(1+0.5)*sqrt(afa_u)*B3,1)
                lmiterm([1 9 3 K],sqrt(1+0.5)*sqrt(afa_u)*C3,1)
                lmiterm([1 9 4 K],sqrt(1+0.5)*sqrt(afa_u)*D3,1)
        
                lmiterm([1 10 1 K],sqrt(1+0.5)*sqrt(beita_u)*A4,1)
                lmiterm([1 10 2 K],sqrt(1+0.5)*sqrt(beita_u)*B4,1)
                lmiterm([1 10 3 K],sqrt(1+0.5)*sqrt(beita_u)*C4,1)
                lmiterm([1 10 4 K],sqrt(1+0.5)*sqrt(beita_u)*D4,1)
        
                lmiterm([1 11 1 K],sqrt(1+0.5)*sqrt(afa_x*beita_x)*A5,1)
                lmiterm([1 11 2 K],sqrt(1+0.5)*sqrt(afa_x*beita_x)*B5,1)
                lmiterm([1 11 3 K],sqrt(1+0.5)*sqrt(afa_x*beita_x)*C5,1)
                lmiterm([1 11 4 K],sqrt(1+0.5)*sqrt(afa_x*beita_x)*D5,1)
        
                lmiterm([1 12 1 K],sqrt(1+0.5)*sqrt(afa_x*afa_u)*A6,1)
                lmiterm([1 12 2 K],sqrt(1+0.5)*sqrt(afa_x*afa_u)*B6,1)
                lmiterm([1 12 3 K],sqrt(1+0.5)*sqrt(afa_x*afa_u)*C6,1)
                lmiterm([1 12 4 K],sqrt(1+0.5)*sqrt(afa_x*afa_u)*D6,1)
        
                lmiterm([1 13 1 K],sqrt(1+0.5)*sqrt(afa_x*beita_u)*A7,1)
                lmiterm([1 13 2 K],sqrt(1+0.5)*sqrt(afa_x*beita_u)*B7,1)
                lmiterm([1 13 3 K],sqrt(1+0.5)*sqrt(afa_x*beita_u)*C7,1)
                lmiterm([1 13 4 K],sqrt(1+0.5)*sqrt(afa_x*beita_u)*D7,1)
        
                lmiterm([1 14 1 K],sqrt(1+0.5)*sqrt(beita_x*afa_u)*A8,1)
                lmiterm([1 14 2 K],sqrt(1+0.5)*sqrt(beita_x*afa_u)*B8,1)
                lmiterm([1 14 3 K],sqrt(1+0.5)*sqrt(beita_x*afa_u)*C8,1)
                lmiterm([1 14 4 K],sqrt(1+0.5)*sqrt(beita_x*afa_u)*D8,1)
        
                lmiterm([1 15 1 K],sqrt(1+0.5)*sqrt(beita_x*beita_u)*A9,1)
                lmiterm([1 15 2 K],sqrt(1+0.5)*sqrt(beita_x*beita_u)*B9,1)
                lmiterm([1 15 3 K],sqrt(1+0.5)*sqrt(beita_x*beita_u)*C9,1)
                lmiterm([1 15 4 K],sqrt(1+0.5)*sqrt(beita_x*beita_u)*D9,1)
        
                lmiterm([1 16 1 K],sqrt(1+0.5)*sqrt(afa_u*beita_u)*A10,1)
                lmiterm([1 16 2 K],sqrt(1+0.5)*sqrt(afa_u*beita_u)*B10,1)
                lmiterm([1 16 3 K],sqrt(1+0.5)*sqrt(afa_u*beita_u)*C10,1)
                lmiterm([1 16 4 K],sqrt(1+0.5)*sqrt(afa_u*beita_u)*D10,1)
        
                lmiterm([1 17 1 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u)*A11,1)
                lmiterm([1 17 2 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u)*B11,1)
                lmiterm([1 17 3 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u)*C11,1)
                lmiterm([1 17 4 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u)*D11,1)
        
                lmiterm([1 18 1 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*beita_u)*A12,1)
                lmiterm([1 18 2 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*beita_u)*B12,1)
                lmiterm([1 18 3 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*beita_u)*C12,1)
                lmiterm([1 18 4 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*beita_u)*D12,1)
        
                lmiterm([1 19 1 K],sqrt(1+0.5)*sqrt(afa_x*afa_u*beita_u)*A13,1)
                lmiterm([1 19 2 K],sqrt(1+0.5)*sqrt(afa_x*afa_u*beita_u)*B13,1)
                lmiterm([1 19 3 K],sqrt(1+0.5)*sqrt(afa_x*afa_u*beita_u)*C13,1)
                lmiterm([1 19 4 K],sqrt(1+0.5)*sqrt(afa_x*afa_u*beita_u)*D13,1)
        
                lmiterm([1 20 1 K],sqrt(1+0.5)*sqrt(beita_u*afa_u*beita_u)*A14,1)
                lmiterm([1 20 2 K],sqrt(1+0.5)*sqrt(beita_u*afa_u*beita_u)*B14,1)
                lmiterm([1 20 3 K],sqrt(1+0.5)*sqrt(beita_u*afa_u*beita_u)*C14,1)
                lmiterm([1 20 4 K],sqrt(1+0.5)*sqrt(beita_u*afa_u*beita_u)*D14,1)
        
                lmiterm([1 21 1 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u*beita_u)*A15,1)
                lmiterm([1 21 2 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u*beita_u)*B15,1)
                lmiterm([1 21 3 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u*beita_u)*C15,1)
                lmiterm([1 21 4 K],sqrt(1+0.5)*sqrt(afa_x*beita_x*afa_u*beita_u)*D15,1)
        
                lmiterm([1 6 6 inv(P)],-1,1)
                lmiterm([1 7 7 inv(P)],-1,1)
                lmiterm([1 8 8 inv(P)],-1,1)
                lmiterm([1 9 9 inv(P)],-1,1)
                lmiterm([1 10 10 inv(P)],-1,1)
                lmiterm([1 12 12 inv(P)],-1,1)
                lmiterm([1 13 13 inv(P)],-1,1)
                lmiterm([1 14 14 inv(P)],-1,1)
                lmiterm([1 15 15 inv(P)],-1,1)
                lmiterm([1 16 16 inv(P)],-1,1)
                lmiterm([1 17 17 inv(P)],-1,1)
                lmiterm([1 18 18 inv(P)],-1,1)
                lmiterm([1 19 19 inv(P)],-1,1)
                lmiterm([1 20 20 inv(P)],-1,1)
                lmiterm([1 21 21 inv(P)],-1,1)
                        
                lmiterm([2 1 1 gama],-eye(2),1)
                lmiterm([2 2 2 gama],-eye(2),1)     
                lmiterm([2 3 3 gama],-eye(1),1)
                lmiterm([2 4 4 gama],-eye(2),1)  
                lmiterm([2 5 5 gama],-eye(2),1)
                lmiterm([2 6 6 gama],-eye(1),1)  
                lmiterm([2 7 7 gama],-eye(2),1)
           
                lmiterm([2 8 1 0],eye(2))
                lmiterm([2 8 2 K],sqrt(1+2)*S,1)
                lmiterm([2 8 3 0],sqrt(1+2)*E)
                lmiterm([2 8 4 K],sqrt(1+2)*F,1)
                lmiterm([2 8 5 K],sqrt(1+2)*G,1)
                lmiterm([2 8 6 0],sqrt(1+2)*H)
                lmiterm([2 8 7 K],sqrt(1+2)*Lt,1)
        
                lmiterm([2 9 2 K],sqrt(1+2)*sqrt(afa_x)*S1,1)
                lmiterm([2 9 4 K],sqrt(1+2)*sqrt(afa_x)*F1,1)
                lmiterm([2 9 5 K],sqrt(1+2)*sqrt(afa_x)*G1,1)
                lmiterm([2 9 7 K],sqrt(1+2)*sqrt(afa_x)*L1,1)
        
                lmiterm([2 10 2 K],sqrt(1+2)*sqrt(beita_x)*S2,1)
                lmiterm([2 10 4 K],sqrt(1+2)*sqrt(beita_x)*F2,1)
                lmiterm([2 10 5 K],sqrt(1+2)*sqrt(beita_x)*G2,1)
                lmiterm([2 10 7 K],sqrt(1+2)*sqrt(beita_x)*L2,1)
        
                lmiterm([2 11 2 K],sqrt(1+2)*sqrt(afa_u)*S3,1)
                lmiterm([2 11 3 0],sqrt(1+2)*sqrt(afa_u)*E3)
                lmiterm([2 11 4 K],sqrt(1+2)*sqrt(afa_u)*F3,1)
                lmiterm([2 11 5 K],sqrt(1+2)*sqrt(afa_u)*G3,1)
                lmiterm([2 11 6 0],sqrt(1+2)*sqrt(afa_u)*H3)
                lmiterm([2 11 7 K],sqrt(1+2)*sqrt(afa_u)*L3,1)
        
                lmiterm([2 12 2 K],sqrt(1+2)*sqrt(beita_u)*S4,1)
                lmiterm([2 12 3 0],sqrt(1+2)*sqrt(beita_u)*E4)
                lmiterm([2 12 4 K],sqrt(1+2)*sqrt(beita_u)*F4,1)
                lmiterm([2 12 5 K],sqrt(1+2)*sqrt(beita_u)*G4,1)
                lmiterm([2 12 6 0],sqrt(1+2)*sqrt(beita_u)*H4)
                lmiterm([2 12 7 K],sqrt(1+2)*sqrt(beita_u)*L4,1)
        
                lmiterm([2 13 2 K],sqrt(1+2)*sqrt(afa_x*beita_x)*S5,1)
                lmiterm([2 13 4 K],sqrt(1+2)*sqrt(afa_x*beita_x)*F5,1)
                lmiterm([2 13 5 K],sqrt(1+2)*sqrt(afa_x*beita_x)*G5,1)
                lmiterm([2 13 7 K],sqrt(1+2)*sqrt(afa_x*beita_x)*L5,1)
        
                lmiterm([2 14 2 K],sqrt(1+2)*sqrt(afa_x*afa_u)*S6,1)
                lmiterm([2 14 4 K],sqrt(1+2)*sqrt(afa_x*afa_u)*F6,1)
                lmiterm([2 14 5 K],sqrt(1+2)*sqrt(afa_x*afa_u)*G6,1)
                lmiterm([2 14 7 K],sqrt(1+2)*sqrt(afa_x*afa_u)*L6,1)
        
                lmiterm([2 15 2 K],sqrt(1+2)*sqrt(afa_x*beita_u)*S7,1)
                lmiterm([2 15 4 K],sqrt(1+2)*sqrt(afa_x*beita_u)*F7,1)
                lmiterm([2 15 5 K],sqrt(1+2)*sqrt(afa_x*beita_u)*G7,1)
                lmiterm([2 15 7 K],sqrt(1+2)*sqrt(afa_x*beita_u)*L7,1)
        
                lmiterm([2 16 2 K],sqrt(1+2)*sqrt(beita_x*afa_u)*S8,1)
                lmiterm([2 16 4 K],sqrt(1+2)*sqrt(beita_x*afa_u)*F8,1)
                lmiterm([2 16 5 K],sqrt(1+2)*sqrt(beita_x*afa_u)*G8,1)
                lmiterm([2 16 7 K],sqrt(1+2)*sqrt(beita_x*afa_u)*L8,1)
        
                lmiterm([2 17 2 K],sqrt(1+2)*sqrt(beita_x*beita_u)*S9,1)
                lmiterm([2 17 4 K],sqrt(1+2)*sqrt(beita_x*beita_u)*F9,1)
                lmiterm([2 17 5 K],sqrt(1+2)*sqrt(beita_x*beita_u)*G9,1)
                lmiterm([2 17 7 K],sqrt(1+2)*sqrt(beita_x*beita_u)*L9,1)
        
                lmiterm([2 18 2 K],sqrt(1+2)*sqrt(afa_u*beita_u)*S10,1)
                lmiterm([2 18 3 0],sqrt(1+2)*sqrt(afa_u*beita_u)*E10)
                lmiterm([2 18 4 K],sqrt(1+2)*sqrt(afa_u*beita_u)*F10,1)
                lmiterm([2 18 5 K],sqrt(1+2)*sqrt(afa_u*beita_u)*G10,1)
                lmiterm([2 18 6 0],sqrt(1+2)*sqrt(afa_u*beita_u)*H10)
                lmiterm([2 18 7 K],sqrt(1+2)*sqrt(afa_u*beita_u)*L10,1)
        
                lmiterm([2 19 2 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u)*S11,1)
                lmiterm([2 19 4 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u)*F11,1)
                lmiterm([2 19 5 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u)*G11,1)
                lmiterm([2 19 7 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u)*L11,1)
        
                lmiterm([2 20 2 K],sqrt(1+2)*sqrt(afa_x*beita_x*beita_u)*S12,1)
                lmiterm([2 20 4 K],sqrt(1+2)*sqrt(afa_x*beita_x*beita_u)*F12,1)
                lmiterm([2 20 5 K],sqrt(1+2)*sqrt(afa_x*beita_x*beita_u)*G12,1)
                lmiterm([2 20 7 K],sqrt(1+2)*sqrt(afa_x*beita_x*beita_u)*L12,1)
        
                lmiterm([2 21 2 K],sqrt(1+2)*sqrt(afa_x*afa_u*beita_u)*S13,1)
                lmiterm([2 21 4 K],sqrt(1+2)*sqrt(afa_x*afa_u*beita_u)*F13,1)
                lmiterm([2 21 5 K],sqrt(1+2)*sqrt(afa_x*afa_u*beita_u)*G13,1)
                lmiterm([2 21 7 K],sqrt(1+2)*sqrt(afa_x*afa_u*beita_u)*L13,1)
        
                lmiterm([2 22 2 K],sqrt(1+2)*sqrt(beita_x*afa_u*beita_u)*S14,1)
                lmiterm([2 22 4 K],sqrt(1+2)*sqrt(beita_x*afa_u*beita_u)*F14,1)
                lmiterm([2 22 5 K],sqrt(1+2)*sqrt(beita_x*afa_u*beita_u)*G14,1)
                lmiterm([2 22 7 K],sqrt(1+2)*sqrt(beita_x*afa_u*beita_u)*L14,1)
        
                lmiterm([2 23 2 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u*beita_u)*S15,1)
                lmiterm([2 23 4 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u*beita_u)*F15,1)
                lmiterm([2 23 5 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u*beita_u)*G15,1)
                lmiterm([2 23 7 K],sqrt(1+2)*sqrt(afa_x*beita_x*afa_u*beita_u)*L15,1)
        
                lmiterm([2 8 8 inv(P)],-1,1)
                lmiterm([2 9 9 inv(P)],-1,1)
                lmiterm([2 10 10 inv(P)],-1,1)
                lmiterm([2 11 11 inv(P)],-1,1)
                lmiterm([2 12 12 inv(P)],-1,1)
                lmiterm([2 13 13 inv(P)],-1,1)
                lmiterm([2 14 14 inv(P)],-1,1)
                lmiterm([2 15 15 inv(P)],-1,1)
                lmiterm([2 16 16 inv(P)],-1,1)
                lmiterm([2 17 17 inv(P)],-1,1)
                lmiterm([2 18 18 inv(P)],-1,1)
                lmiterm([2 19 19 inv(P)],-1,1)
                lmiterm([2 20 20 inv(P)],-1,1)
                lmiterm([2 21 21 inv(P)],-1,1)
                lmiterm([2 22 22 inv(P)],-1,1)
                lmiterm([2 23 23 inv(P)],-1,1)
        
                lmiterm([3 1 1 P],-1,1)
                lmiterm([4 1 1 Q1],-1,1)
                lmiterm([5 1 1 Q2],-1,1)
                lmiterm([6 1 1 Q3],-1,1)
                lmiterm([7 1 1 miu],-1,1)
                lmiterm([8 1 1 gama],-1,1)
                lmiterm([9 1 1 beita],-1,1)
                
                lmis=getlmis;
                % s=eig(A);
                [tmin,xfeas]=feasp(lmis);
            
                P=dec2mat(lmis,xfeas,P);
                Q1=dec2mat(lmis,xfeas,Q1);
                Q2=dec2mat(lmis,xfeas,Q2);
                Q3=dec2mat(lmis,xfeas,Q3);
                K=dec2mat(lmis,xfeas,K);
                miu=dec2mat(lmis,xfeas,miu);
                beita=dec2mat(lmis,xfeas,beita);
                gama=dec2mat(lmis,xfeas,gama);
                kexi(k) = K*x_n-3.5; 
            else
                kexi(k) = K*x_n*1.1; % 检测攻击后第一次的传输不会被攻击
            end
            %% 执行器部分
            u4(:,k) = (1-c_a_DoSattack4(k))*(kexi(k)+c_a_decattack4(k)*(-kexi(k)+wu(k))) ... 
                +c_a_DoSattack4(k)*(kexik4(tao_uk(k))+c_a_decattack4(k)*(-kexik4(tao_uk(k))+wuk4(tao_uk(k))));
    
            event4(k) = 3;
        else
            event4(k) = 0;
            e4(:,k) = xjk4-x4(:,k);
            kexi(k) = kexi(k-1);
            u4(:,k) = u4(:,k-1);
        end
        
        J4(k) = x4(:,k)'*Q*x4(:,k)+u4(k)'*R*u4(k);
        x4(:, k+1) = Ar*x4(:, k) + Br*u4(k)+f4(:,k)+[0;0.036*sin(k*pi/3)]; 
        sum_e4 = sum_e4+x4(:, k)'*x4(:, k);% 计算综合误差
        zeta4 = theta4*zeta4+0.3*norm(x4(:,k),2)^2-norm(x4(:,k)-xjk4,2)^2; % 动量因子迭代公式
    
        %% 数据更新
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
        kexik4(1) = kexi(:,k);
        wuk4(:,1) = wu(:,k);
        
    end
    
    sum_t4=L/(sum(event4)/3);
    % sum_t4=(sum(event4)/3);
    sum_J4 = sum(J4);% 计算性能指标
    
    %% DET-MPC
  
    %% 历史时刻的状态和控制作用
    x(:,1)=[-1.2;1.2];% 系统状态参数初值
    xp(:,1) = [-1.2;1.2];% 判别模型的初值
    X=[2;2];% 状态向量的最大值
    xk = zeros(2,tao_x);  % 系统状态的过去值
    ek = zeros(2,tao_x);  % 触发误差的过去值
    kexik = zeros(1,tao_u);   % 控制器输出的过去值
    wxk = zeros(2,tao_x);     % 传感器到控制器扰动的过去值
    wuk = zeros(1,tao_u);     % 控制器到执行器扰动的过去值
    xjk = zeros(2,1);% 上一触发时刻得输出值
    zeta = 1;            % 动态事件触发的动量因子ζ
    theta = 0.5;         % 动量因子迭代公式θ   
    deta = 6;            % 动态事件触发迭代因子δ
    sum_e = 0;
    
    %% 执行运算
    for k=1:L
        H=2*Bt'*Qt*Bt+2*Rt;
        f(:,k)=[0;-0.216*exp(-x(1,k))*x(1,k)-0.048*cos(k*pi/5)*x(2,k)];
        e(:,k) = xjk-x(:,k);
        %%  触发时刻重新计算控制作用u(k)
        if norm(x(:,k)-xjk,2)^2 >= 0.1*norm(x(:,k),2)^2+1/deta*zeta
%         if norm(x(:,k)-xjk,2)^2 >= eipilon
            xjk = x(:,k);% 更新上一触发时刻的状态
            e(:,k) = xjk-x(:,k);
    %         tao_xk(1,k) = round(rand(1,1)*(tao_x-1))+1;
            x_n = (1-s_c_DoSattack(k))*(x(:,k)+e(:,k)+s_c_decattack(k)*(-2*x(:,k)-2*e(:,k)+wx(:,k))) ... 
                +s_c_DoSattack(k)*(xk(:,tao_xk(k))+ek(:,tao_xk(k))+s_c_decattack(k)*(-2*xk(:,tao_xk(k)) ...
                -2*ek(:,tao_xk(k))+wxk(:,tao_xk(k))));% 存在欺骗攻击
            xp(:,k+1) = x_n;
    
            %% 控制器部分
            f_p(:,k)=[0;-0.216*exp(-xp(1,k+1))*xp(1,k+1)-0.048*cos(k*pi/5)*xp(2,k+1)];   
            ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp(:,k+1)+Ft*f_p(:,k));
            
            xp(:,k+1) = Ar*xp(:,k+1)+Br*ut(1)+f_p(:,k);
            kexi(k) = ut(1); % 检测攻击后第一次的传输不会被攻击
    
            %% 执行器部分
            u(:,k) = (1-c_a_DoSattack(k))*(kexi(k)+c_a_decattack(k)*(-kexi(k)+wu(k))) ... 
                +c_a_DoSattack(k)*(kexik(tao_uk(k))+c_a_decattack(k)*(-kexik(tao_uk(k))+wuk(tao_uk(k))));
    %         tao_uk(1,k) = round(rand(1,1)*(tao_u-1))+1;
            event(k) = 2;
        else
            event(k) = 0;
            e(:,k) = xjk-x(:,k);
            kexi(k) = ut(1);
            f_p(:,k)=[0;-0.216*exp(-xp(1,k))*xp(1,k)-0.048*cos(k*pi/5)*xp(2,k)];  
    %         ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp(:,k)+Ft*f_p(:,k));
            xp(:,k+1) = Ar*xp(:,k)+Br*ut(1)+f_p(:,k);
            u(:,k) = u(:,k-1);
        end
    
        J(k) = x(:,k)'*Q*x(:,k)+u(k)'*R*u(k);
        x(:, k+1) = Ar*x(:, k) + Br*u(k)+f(:,k)+[0;0.036*sin(k*pi/3)]; 
        sum_e = sum_e+x(:, k)'*x(:, k);% 计算综合误差
        zeta = theta*zeta+0.1*norm(x(:,k),2)^2-norm(x(:,k)-xjk,2)^2; % 动量因子迭代公式
    %     zeta = theta*zeta+eipilon-norm(x(:,k)-xjk,2)^2; % 动量因子迭代公式
        
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
    
    sum_t=L/(sum(event)/2);
    % sum_t=(sum(event)/2);
    sum_J = sum(J);% 计算性能指标
    
    %% proposed method
    
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
            if norm((xp1(:,k)-x_n),2)^2 >= 0.8
                xp1(:,k+1) = xp1(:,k);
                detector1(k) = 1; % 有发生攻击
            else
                xp1(:,k+1) = x_n;
                detector1(k) = 0; % 有发生攻击
            end
    
            %% 控制器部分
            f_p1(:,k)=[0;-0.216*exp(-xp1(1,k+1))*xp1(1,k+1)-0.048*cos(k*pi/5)*xp1(2,k+1)];   
            ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp1(:,k+1)+Ft*f_p1(:,k));
            
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
    
    sum_t1=L/sum(event1);
    % sum_t1=sum(event1);
    sum_J1 = sum(J1);% 计算性能指标

     %% 历史时刻的状态和控制作用
    x2(:,1)=[-1.2;1.2];% 系统状态参数初值
    uk2 = [-0.3169,-1.1566]*x2(:,1); % 控制器上一可接受初值
    xk2 = zeros(2,tao_x);  % 系统状态的过去值
    ek2 = zeros(2,tao_x);  % 触发误差的过去值
    kexik2 = zeros(1,tao_u);   % 控制器输出的过去值
    wxk2 = zeros(2,tao_x);     % 传感器到控制器扰动的过去值
    wuk2 = zeros(1,tao_u);     % 控制器到执行器扰动的过去值
    xjk2 = zeros(2,1);% 上一触发时刻得输出值
    zeta2 = 1;            % 动态事件触发的动量因子ζ
    theta2 = 0.5;         % 动量因子迭代公式θ   
    deta2 = 6;            % 动态事件触发迭代因子δ
    sum_e2 = 0;
    
    %% 判别模型
    xp2(:,1) = [-1.2;1.2];% 判别模型的初值
    xpk2 = zeros(2,tao_x);  % 系统状态的过去值
    x_store2(:,1) = [-1.2;1.2];% 控制器端存储可行状态的初值
    xp_store2(:,1) = [-1.2;1.2];% 控制器端存储可行状态的初值
    m1k = [0.0;0.0];
    m1_pk = [0.0;0.0];
    
    %% 执行运算
    for k=1:L
        H=2*Bt'*Qt*Bt+2*Rt;
        f2(:,k)=[0;-0.216*exp(-x2(1,k))*x2(1,k)-0.048*cos(k*pi/5)*x2(2,k)];
        fp2(:,k)=[0;-0.216*exp(-xp2(1,k))*xp2(1,k)-0.048*cos(k*pi/5)*xp2(2,k)];
        e2(:,k) = xjk2-x2(:,k);
        %%  触发时刻重新计算控制作用u(k)
%         if norm(x(:,k)-xjk,2)^2 >= 0.3*norm(x(:,k),2)^2
        if norm(x(:,k)-xjk,2)^2 >= eipilon+1/deta2*zeta2
%         if norm(x(:,k)-xjk,2)^2 >= eipilon
            xjk2 = x2(:,k);% 更新上一触发时刻的状态
            e2(:,k) = xjk2-x2(:,k);
    %         tao_xk(1,k) = round(rand(1,1)*(tao_x-1))+1;
            x_n = (1-s_c_DoSattack(k))*(x2(:,k)+e2(:,k)+s_c_decattack(k)*(-2*x2(:,k)-2*e2(:,k)+wx(:,k))) ... 
                +s_c_DoSattack(k)*(xk2(:,tao_xk(k))+ek2(:,tao_xk(k))+s_c_decattack(k)*(-2*xk2(:,tao_xk(k)) ...
                -2*ek2(:,tao_xk(k))+wxk2(:,tao_xk(k))));% 存在欺骗攻击
            x_p = (1-s_c_DoSattack1(k))*(xp2(:,k)+e2(:,k)+s_c_decattack1(k)*(-2*xp2(:,k)-2*e2(:,k)+wx(:,k))) ... 
                +s_c_DoSattack1(k)*(xpk2(:,tao_xk(k))+ek2(:,tao_xk(k))+s_c_decattack1(k)*(-2*xpk2(:,tao_xk(k)) ...
                -2*ek2(:,tao_xk(k))+wxk2(:,tao_xk(k))));% 存在欺骗攻击
            
            if ((norm((x_store2(:,k)-x_n),2)^2 > 0.8) || ...
                    (norm((x_store2(:,k)-x_p),2)^2 > 0.1)) % && mod(k,20) ~= 0
                x_store2(:,k) = x_store2(:,k);
                detector(k) = 2; % 有发生攻击
            %         s_c_attack1(k+1) = 0;
            else
                x_store2(:,k) = x_n;
                %         x_store1(:,k+1) = x_store1(:,k);
                detector(k) = 0; % 没有发生攻击
            end
            
            %% 控制器部分
            f_s2(:,k)=[0;-0.216*exp(-x_store2(1,k))*x_store2(1,k)-0.048*cos(k*pi/5)*x_store2(2,k)];   
            ut2 = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*x_store2(:,k)+Ft*f_s2(:,k));
            K =  -(Bt'*Qt*Bt+Rt)\Bt'*Qt*At;
            x_store2(:,k+1) = Ar*x_store2(:,k)+Br*ut2(1)+f_s2(:,k);
            
            kexi2(k) = ut2(1); % 检测攻击后第一次的传输不会被攻击
    
            %% 执行器部分
    %         tao_uk(1,k) = round(rand(1,1)*(tao_u-1))+1;
    %         if detector(k) == 0
                u2(:,k) = (1-c_a_DoSattack(k))*(kexi2(k)+c_a_decattack(k)*(-kexi2(k)+wu(k))) ... 
                    +c_a_DoSattack(k)*(kexik2(tao_uk(k))+c_a_decattack(k)*(-kexik2(tao_uk(k))+wuk2(tao_uk(k))));
    %         else
    %             u(:,k) = kexi(k);
    %         end
            event2(k) = 4;
        else
            event2(k) = 0;
            detector(k) = 0; % 没有发生攻击
            e(:,k) = xjk-x(:,k);
            kexi2(k) = ut2(1);
            f_s(:,k)=[0;-0.216*exp(-x_store2(1,k))*x_store2(1,k)-0.048*cos(k*pi/5)*x_store2(2,k)];
    %         ut = -(Bt'*Qt*Bt+Rt)\Bt'*Qt*(At*xp(:,k)+Ft*f_p(:,k));
            x_store2(:,k+1) = Ar*x_store2(:,k)+Br*ut(1)+f_s(:,k);
            u2(:,k) = u2(:,k-1);
        end
    
        J2(k) = x2(:,k)'*Q*x2(:,k)+u2(k)'*R*u2(k);
        if detector(k) == 1
            x2(:, k+1) = Ar*x2(:, k) + Br*uk2+f2(:,k); 
            xp2(:,k+1) = Ar*xp2(:, k) + Br*u2(:,k)+fp2(:,k);
            uk2 = ut2(1); 
        else
            x2(:, k+1) = Ar*x2(:, k) + Br*uk2+f2(:,k); 
            xp2(:,k+1) = Ar*xp2(:, k) + Br*u2(:,k)+fp2(:,k); 
            uk2 = u2(k);
        end
        sum_e2 = sum_e2+x2(:, k)'*x2(:, k);% 计算综合误差
    %     zeta = theta*zeta+0.3*norm(x(:,k),2)^2-norm(x(:,k)-xjk,2)^2; % 动量因子迭代公式
        zeta2 = theta2*zeta2+eipilon-norm(x2(:,k)-xjk2,2)^2; % 动量因子迭代公式
    
        %% 数据更新
        for i=tao_x:-1:2
            xk2(:,i) = xk2(:,i-1);
            xpk2(:,i) = xpk2(:,i-1);
            ek2(:,i) = ek2(:,i-1);
            wxk2(:,i) = wxk2(:,i-1);
        end
        xk2(:,1) = x2(:,k);
        xpk2(:,1) = xp2(:,k);
        ek2(:,1) = e2(:,k);
        wxk2(:,1) = wx(:,k);
        for i=tao_u:-1:2
            kexik2(i) = kexik2(i-1);
            wuk2(:,i) = wuk2(:,i-1);
        end
        kexik2(1) = kexi2(:,k);
        wuk2(:,1) = wu(:,k);
        
    end
    
    sum_t2=L/(sum(event)/4);
    sum_J2 = sum(J2);% 计算性能指标

%     if sum_J1 <= sum_J && sum_J1 <= sum_J4 && sum_J1 <= sum_J2 && sum_J1 <= 10 && sum_J2 <= 15 && sum_J <= 20 && sum_J4 <= 25
%     if sum_e1 <= sum_e && sum_e1 <= sum_e4 && sum_e1 <= sum_e2 && sum_e1 <= 15 && sum_e2 <= 15 && sum_e <= 20 && sum_e4 <= 28
    if sum_e1 <= sum_e && sum_e1 <= 10 && sum_e <= 20
        success = success+1;
        break;
    end

end

% %% 仿真结果
% figure(1)
% subplot(2,1,1)
% i=1:L+1;
% plot(i,x4(1,i),'--b',i,x(1,i),'--r',i,x2(1,i),'--m',i,x1(1,i),'k','linewidth',2);
% % xlabel('k/step');
% ylabel('x_{1,k}');
% % title('State output of comparison');
% legend('DET-feedback control x_{1,k}','DET-MPC x_{1,k}','DET-detect-MPC x_{1,k}','Proposed method x_{1,k}');
% grid on;
% axis([0 L -2 3]);
% subplot(2,1,2)
% i=1:L+1;
% plot(i,x4(2,i),'--b',i,x(2,i),'--r',i,x2(2,i),'--m',i,x1(2,i),'k','linewidth',2);
% xlabel('k/step');
% ylabel('x_{2,k}');
% legend('DET-feedback control x_{2,k}','DET-MPC x_{2,k}','DET-detect-MPC x_{1,k}','Proposed method x_{2,k}');
% grid on;
% axis([0 L -2 3]);
% 
% figure(2)
% subplot(4,1,1)
% i=1:L;
% b=bar(i,s_c_decattack(1,i),1,'b');
% set(b,'edgecolor','none')
% ylabel('\alpha_{k}^x');
% % title('s-c deception attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;
% subplot(4,1,2)
% i=1:L;
% b=bar(i,s_c_DoSattack(1,i),1,'b');
% set(b,'edgecolor','none')
% ylabel('\beta_{k}^x');
% % title('s-c DoS attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;
% subplot(4,1,3)
% i=1:L;
% b=bar(i,c_a_decattack(1,i),1,'b');
% set(b,'edgecolor','none')
% ylabel('\alpha_{k}^u');
% % title('c-a deception attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;
% subplot(4,1,4)
% i=1:L;
% b=bar(i,c_a_DoSattack(1,i),1,'b');
% set(b,'edgecolor','none')
% xlabel('k/step');
% ylabel('\beta_{k}^u');
% % title('c-a DoS attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;
% 
% figure(3)
% subplot(2,1,1)
% i=1:L;
% plot(i,u4(1,i),'--b',i,u(1,i),'--r',i,u2(1,i),'--m',i,u1(1,i),'k','linewidth',2);
% % xlabel('k/step');
% ylabel('u_k');
% % title('Control law comparison');
% legend('DET-feedback u_k','DET-MPC u_k','DET-detect-MPC u_k','Proposed method u_k');
% grid on;
% axis([0 L -4 6]);
% subplot(2,1,2)
% i=1:L;
% sz = 50;
% scatter(i,event1,sz,'d');
% hold on;
% scatter(i,event2,sz,'s');
% hold on;
% scatter(i,event,sz,'h');
% hold on;
% scatter(i,event4,sz,'o');
% hold on;
% xlabel('k/step');
% % ylabel('The triggering instants');
% yticks([1 2 3 4])
% yticklabels({'Proposed method','DET-detect-MPC','DET-MPC','DET-feedback'})
% grid on;
% axis([0 L 1 4]);

%% 仿真结果
figure(1)
subplot(3,1,1)
i=1:L+1;
plot(i,x(1,i),'--r',i,x1(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{1,k}');
% title('State output of comparison');
legend('DET-MPC x_{1,k}','Proposed method x_{1,k}');
grid on;
axis([0 L -2 2]);
subplot(3,1,2)
i=1:L+1;
plot(i,x(2,i),'--r',i,x1(2,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('x_{2,k}');
legend('DET-MPC x_{2,k}','Proposed method x_{2,k}');
grid on;
axis([0 L -1.5 2.5]);
subplot(3,1,3)
i=1:L;
b=bar(i,c_a_decattack(1,i)+s_c_decattack(1,i)+c_a_DoSattack(1,i)+s_c_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
xlabel('k/step');
ylabel('attack');
axis([0 L 0 1]);
% title('c-a deception attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;

% figure(2)
% subplot(2,1,1)
% i=1:L;
% b=bar(i,c_a_decattack(1,i),1,'b');
% set(b,'edgecolor','none')
% ylabel('\alpha_{k}^u');
% % title('c-a deception attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;
% subplot(2,1,2)
% i=1:L;
% b=bar(i,c_a_DoSattack(1,i),1,'b');
% set(b,'edgecolor','none')
% xlabel('k/step');
% ylabel('\beta_{k}^u');
% % title('c-a DoS attack');
% % legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
% grid on;

figure(3)
subplot(3,1,1)
i=1:L;
plot(i,u(1,i),'--r',i,u1(1,i),'k','linewidth',2);
% xlabel('k/step');
ylabel('u_k');
% title('Control law comparison');
legend('DET-MPC u_k','Proposed method u_k');
grid on;
axis([0 L -4 3]);
subplot(3,1,2)
i=1:L;
b=bar(i,c_a_decattack(1,i)+s_c_decattack(1,i)+c_a_DoSattack(1,i)+s_c_DoSattack(1,i),1,'b');
set(b,'edgecolor','none')
xlabel('k/step');
ylabel('attack');
axis([0 L 0 1]);
% title('c-a deception attack');
% legend('DET-feedback u_k','DET-MPC u_k','Proposed method u_k');
grid on;
subplot(3,1,3)
i=1:L;
sz = 50;
scatter(i,event1,sz,'d');
hold on;
scatter(i,event,sz,'h');
hold on;
% scatter(i,event4,sz,'o');
hold on;
xlabel('k/step');
% ylabel('The triggering instants');
yticks([1 2 3])
yticklabels({'Proposed method','DET-MPC'})
grid on;
axis([0 L 1 3]);

