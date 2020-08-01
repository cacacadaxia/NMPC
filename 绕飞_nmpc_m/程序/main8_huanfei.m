clc;
clear all;
warning('off');

%%
%Spacecraft initial state parameters
Mu=4.794e3;%引力常数
re =1750;%切面半径
c20=-0.113;
c22=0.0396;
% xit0=0;%着陆点赤经
% fai0=pi/2;%着陆点赤纬
W =2*pi/5.27/3600;%小行星自旋角速度
% Gl=[-cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    cos(fai0);
%     -sin(fai0)               -cos(xit0)            0    ;
%     cos(xit0)*cos(fai0)  -sin(xit0)*cos(fai0)    sin(fai0)];
Gl=[1 0 0;0 1 0;0 0 1];
We=Gl'*[0;0;W];
Wex=We(1);
Wey=We(2);
Wez=We(3);

R0l =5830;%小天体质心到着陆点的距离,这个量和平面结合起来。
rol=[0;0;R0l];%小天体质心指向着陆点的向量在着陆坐标系L系的向量
AA=[Wex^2-W^2   Wex*Wey    Wex*Wez;
    Wex*Wey    Wey^2-W^2   Wey*Wez;
    Wex*Wez     Wey*Wez   Wez^2-W^2];%叉乘矩阵
BB=[   0    -2*Wez    2*Wey;
    2*Wez     0     -2*Wex;
    -2*Wey   2*Wex      0  ];%叉乘矩阵

global Mu re c20 c22


%% 给出初始参数
% r0=[-12507;-12000;-9500];
r0=[20000;-20000;0];
v0=[-1.0;0.1;-0.3];
rf=[1507;9870;-1500];%终端位置


%%
%Spacecraft parameters and Time simulation parameters:
M=100;
T=4000;
zeta=2500;

p=40;%预测周期;重要参数。
m=40;%控制周期;

u_max=ones(3,1);
u_min=-ones(3,1);
%%  %模拟的量
omega=2*pi/5.27/3600;%小行星自旋角速度
ww=get_U(Gl*(r0+rol));%获得球谐引力系数
Ac  =[zeros(3) eye(3);
    [omega^2+ww(1)  0  0;
    0  omega^2+ww(2)  0;
    0  0  ww(3)                ] [0 2*omega 0;
    -2*omega 0 0;
    0    0     0]];
Bc=[zeros(3);eye(3)];
Cc=[eye(3) zeros(3)];
Dc=zeros(3);
[A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,10);%temple time is 0.05s

N=330;
global N;


ccc=zeros(6,N);%存储量
ddd=zeros(3,N);%存储量
eee=zeros(6,N);%存储量
fff=zeros(3,N);%存储量
ggg=zeros(3,N);%存储量
hhh=zeros(6,N);
iii=zeros(6,N);

%% input observer
H0=[zeros(3)  eye(3)];
gama=0.025;

%%  %main%
count=0;
fuel=0;
precision=0;

figure;%%%%%
ellipsoid(0,0,0,22000,10000,10000,20);%%%%%
 alpha(.33);%%%%%
grid on;%三维的画线5%%%%%

for i=1:N
    R=zeros(3*p,1);%得到期望轨迹
    [rou,non]=hyperplanes(r0);%rou是距离当前位置超平面上最近的点，non是法向量
%     
%     if dot(r0-rou,rf-rou)<=0
%         rp=projection(non,rf,rou);%获得一个先验作用的终端位置
%     else
%         rp=rf;
%     end
%     norm(rp-rf)
    
    for j=1:p
        tra=trajectory(i+j);%采样周期为0.05,意思说循环四千下其实就走了200秒
%         R(3*j+1:3*j+3,:)=tra(1:3);
%             R(3*j+1:3*j+3,:)=[6507;1000;9500];
            R(3*j+1:3*j+3,:)=rf;%改变终端约束

    end
    tra=trajectory(i+p);

    

    %%
    cvx_begin quiet
        variable XX((p+1)*6+3*m,1) %双变量
        
        
        a=0;
        for q=1:m
%             a=a+norm(XX(6*(p+1)+3*q-2:6*(p+1)+3*q));%本来是应该加权控制量的，这是2范数
                a=a+1*XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%这是2范数的平方
        end
        
        for r=1:p
%             a=a+norm(XX(6*r-5:6*r-3)-R(end-2:end));%这个约束是在加上了非终端状态与终端位置相减的加权
%                  a=a+1*(XX(6*r-5:6*r-3)-R(end-2:end))'*(XX(6*r-5:6*r-3)-R(end-2:end));%2范数的平方
   %为啥加了非终端约束之后，优化器就很容易得到nan的解呢？
        end
        c=1e-2;%对于终端约束的加权
%         a=a+norm(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%保持终端约束，2范数
            a=a+c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%2范数的平方
            
        
    minimize(a);%主要的评价函数就是燃料的加权再加上终端位置的加权
    subject to
        XX(1:6)==[r0;v0];
        for k=1:p
            XX(6*k+1:6*k+6)==A*XX(6*k-5:6*k)+B*XX(6*(p+1)+3*k-2:6*(p+1)+3*k);
            norm(XX(6*k+4:6*k+6))<=40;%对于速度的约束
            non'*(XX(6*k+1:6*k+3)-rou)>=0;%超平面约束
        end
        
%      XX(6*(p+1)-5:6*(p+1)-3)==R(end-2:end);
        abs(XX(6*(p+1)+1:6*(p+1)+3*m))<=1*ones(3*m,1);
        
    cvx_end
    
    %% 调试的量,增加的量后面有5个百分号
%     a%%%%%
%     a_without_u=c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))%%%%%
%     xf=XX(6*(p+1)-5:6*(p+1)-3)%%%%%
%     
%     a_safe=22000;
%     b_safe=10000;
%     c_safe=10000;
%     if xf(1)^2/a_safe^2+xf(2)^2/b_safe^2+xf(3)^2/c_safe^2>=0.99
%         fprintf('%s','满足避障约束');
%     else
%         fprintf('%s','不满足避障约束');
%     end
%     
%     hold on
%     plot3(xf(1),xf(2),xf(3),'p');
%     
    %%
     
    uu=XX(6*(p+1)+1:6*(p+1)+3);
    r_temp=r0;%%%%%%
    sim('dynamic_for_landing',10);
    get_the_anwser=simout(end,:)';%dynamic
    r0=get_the_anwser(1:3)%+0.4*randn(3,1)
    v0=get_the_anwser(4:6);%+0.03*randn(3,1);
    r_change=r0-r_temp%%%%%
    
%     ww=get_U(Gl*(r0+rol));%获得球谐引力系数
%     A  =[zeros(3) eye(3);
%         [omega^2+ww(1)  0  0;
%         0  omega^2+ww(2)  0;
%         0  0  ww(3)] [0 2*omega 0;
%         -2*omega 0 0;
%         0    0     0]];
%     [A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,1);
    
    
    
% ccc(1:3,i)= w_estimate;%记录数据
    ddd(:,i)=v0;%记录数据
    eee(:,i)=trajectory(i);%记录数据
% fff(:,i)=detu;%记录数据
    ggg(:,i)=uu;
    hhh(:,i)=[r0;v0]-trajectory(i);
    iii(:,i)=[r0;v0]+[0;0;0;0;0;0];
    fuel=fuel+norm(uu)*10;
    precision=precision+norm(XX(6*p+1:6*p+3));
    count=count+1
end

%% 附加量：类似于燃料和能量消耗。

fuel
precision

% plot_huanfei;







