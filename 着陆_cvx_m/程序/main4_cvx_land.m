clc;
clear all;
warning('off');
%% some bullshit
Mu=4.794e3;%引力常数
re =1750;%切面半径
c20=-0.113;
c22=0.0396;
xit0 = 1.4785;    % 经度
fai0 = 0;         % 纬度
W =2*pi/5.27/3600;%小行星自旋角速度
Gl=[cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    -cos(fai0);
    -sin(xit0)               cos(xit0)            0    ;
    cos(xit0)*cos(fai0)   sin(xit0)*cos(fai0)    sin(fai0)]';%Cal
We=Gl'*[0;0;W];Wex=We(1);Wey=We(2);Wez=We(3);
R0l =5423.1;%小天体质心到着陆点的距离
rol=[0;0;R0l];%着陆坐标系
AA=[Wex^2-W^2   Wex*Wey    Wex*Wez;
    Wex*Wey    Wey^2-W^2   Wey*Wez;
    Wex*Wez     Wey*Wez   Wez^2-W^2];%叉乘矩阵
BB=[   0    -2*Wez    2*Wey;
    2*Wez     0     -2*Wex;
    -2*Wey   2*Wex      0  ];%叉乘矩阵

global Mu re c20 c22
%%
%% 给出初始参数
% r0=[1507;9870;-1500];%在小天体固连坐标系
r0=[1500;-590.913844154563;4543.78365683008];%着陆坐标系
v0=[-1.0;0.1;-0.3];
%终端
%rrf=[500;5400;0];%小天体固连坐标系
%rrf=[0;0;0];%着陆坐标系,着陆就是从r0到rff的过程。

%%
p=20;%预测周期;重要参数。
m=p;%控制周期;
delta=2;%步长
N=400;
global N

%% 模拟的量  利用差分方程预测动力学
omega=2*pi/5.27/3600;%小行星自旋角速度
ww=Gl'*get_U(Gl*(r0+rol));%获得球谐引力系数
Ac  =[zeros(3) eye(3);
    [omega^2+ww(1)  0  0;
    0  omega^2+ww(2)  0;
    0  0  ww(3)                ] [0 2*omega 0;
    -2*omega 0 0;
    0    0     0]];
Bc=[zeros(3);eye(3)];
Cc=[eye(3) zeros(3)];
Dc=zeros(3);
[A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,delta);%temple time is 0.05s
ccc=zeros(6,N);ddd=zeros(3,N);eee=zeros(6,N);fff=zeros(3,N);ggg=zeros(3,N);hhh=zeros(6,N);iii=zeros(6,N);jjj=zeros(6,N);

%% 抛物约束
Nn=[3;3;0.5];%小天体固连坐标系下
xit1=0.7854;%这个是哪里来的？？
fai1=0.1173;
Gn=[cos(xit1)*sin(fai1)   sin(xit1)*sin(fai1)    -cos(fai1);
    -sin(xit1)               cos(xit1)            0    ;
    cos(xit1)*cos(fai1)   sin(xit1)*cos(fai1)    sin(fai1)]';%G小天体固连坐标系，n系表面坐标系
Cln=Gl'*Gn;
rf_l=[0;0;0];%在着陆坐标系下的终端位置。
dp=150;%安全距离
ap=400;%用来定义抛物面约束的性质
% rp=rf_l-dp*(Nn/norm(Nn));
rp=rf_l-dp*(Cln*[0;0;1]);%着陆系下
%% l系是着陆坐标系，n系是表面坐标系
%% 论文里面用的固联坐标系，但是我觉得用着陆坐标系貌似更加合理

%% input observer
%%  %main%
count=0;
fuel=0;
precision=0;
for i=1:N
    R=zeros(3*(p+1),1);%得到期望
    
    PP=diag([15,25,20,25,15,45]);%blkdiag终端
    %     QQ=diag([15,25,20,25,15,45]);%状态
    QQ=1e-2*PP;
    %     QQ=zeros(6);
    RR=10*eye(3);%控制
    cvx_begin quiet
    variable XX((p+1)*6+3*m,1) %双变量
    a=0;
    %% 主文献里的参数
    PP=1e-3*blkdiag(diag([1,2,5]),2*eye(3));
    QQ=0.001*PP;
    RR=10*eye(3);
    for q=1:m
        a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*RR*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%这是2范数的平方
    end
    for r=1:4
        a=a+(XX(6*r-5:6*r)-R(end-5:end))'*QQ*(XX(6*r-5:6*r)-R(end-5:end));%2范数的平方，过程状态评价函数
    end
    a=a+(XX(6*(p+1)-5:6*(p+1))-R(end-5:end))'*PP*(XX(6*(p+1)-5:6*(p+1))-R(end-5:end));%2范数的平方
    %% 2017里的参数
    %         for q=1:m
    %             a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*RR*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%这是2范数的平方
    %         end
    %         for r=1:p
    %             a=a+(XX(6*r-5:6*r)-R(end-5:end))'*QQ*(XX(6*r-5:6*r)-R(end-5:end));%2范数的平方，过程状态评价函数
    %         end
    %         a=a+(XX(6*(p+1)-5:6*(p+1))-R(end-5:end))'*PP*(XX(6*(p+1)-5:6*(p+1))-R(end-5:end));%2范数的平方
    %% 之前可以用的一些评价函数
    %         for q=1:m
    % %             a=a+norm(XX(6*(p+1)+3*q-2:6*(p+1)+3*q));%本来是应该加权控制量的，这是2范数
    %                 a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%这是2范数的平方
    %         end
    %         for r=1:p
    % %             a=a+norm(XX(6*r-5:6*r-3)-R(end-2:end));%这个约束是在加上了非终端状态与终端位置相减的加权
    % %                  a=a+0.01*(XX(6*r-5:6*r-3)-R(end-2:end))'*(XX(6*r-5:6*r-3)-R(end-2:end));%2范数的平方
    %    %为啥加了非终端约束之后，优化器就很容易得到nan的解呢？
    %         end
    %
    %         c=1e-4;
    % %         a=a+norm(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%保持终端约束，2范数
    %         a=a+c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%2范数的平方
    %%
    minimize(a);%主要的评价函数就是燃料的加权再加上终端位置的加权，这个涉及到具体的加权矩阵的值是多少？
    subject to
    XX(1:6)==[r0;v0];
    for k=1:p
        XX(6*k+1:6*k+6)==A*XX(6*k-5:6*k)+B*XX(6*(p+1)+3*k-2:6*(p+1)+3*k);
        norm(XX(6*k+4:6*k+6))<=10;%对于速度的约束
        (XX(6*k+1:6*k+3)-rp)'*Cln*[1/ap^2  0  0;0  1/ap^2  0;0  0  0]*Cln'*(XX(6*k+1:6*k+3)-rp)-[0;0;1]'*Cln'*(XX(6*k+1:6*k+3)-rp)<=0
        %抛物面约束
    end
    
    for k=1:4
        [0;0;1]'*XX(6*k+1:6*k+3)>=0;
    end
    %      XX(6*(p+1)-5:6*(p+1)-3)==R(end-2:end);
    abs(XX(6*(p+1)+1:6*(p+1)+3*m))<=0.15*ones(3*m,1);
    cvx_end
    
    %% 调试的量,增加的量后面有5个百分号
    %     a%%%%%
    %     a_without_u=c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))%%%%%
    %     xf=XX(6*(p+1)-5:6*(p+1)-3)%%%%%
    %     if (r0-rp)'*Cln*[1/ap^2  0  0;0  1/ap^2  0;0  0  0]*Cln'*(r0-rp)-[0;0;1]'*Cln'*(r0-rp)<=0
    %         fprintf('满足约束\n')
    %     else
    %         fprintf('shit mother fuck\n')
    %     end
    %%
    uu=XX(6*(p+1)+1:6*(p+1)+3)
    
    sim('dynamic_for_landing1',delta);
    RV_result=simout(end,:)';%dynamic
    r0=RV_result(1:3)%+0.4*randn(3,1)
    v0=RV_result(4:6)%+0.03*randn(3,1);
    
    %     ww=Gl'*get_U(Gl*(r0+rol));%获得球谐引力系数
    %     Ac  =[zeros(3) eye(3);
    %         [omega^2+ww(1)  0  0;
    %         0  omega^2+ww(2)  0;
    %         0  0  ww(3)] [0 2*omega 0;
    %         -2*omega 0 0;
    %         0    0     0]];
    %     [A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,delta);
    
    ddd(:,i)=v0;%记录数据
    ggg(:,i)=uu;
    iii(:,i)=[r0;v0];
    jjj(1:3,i)=Gl*(iii(1:3,i)+rol);
    fuel=fuel+norm(uu,1)*delta;
    precision=precision+norm(XX(6*p+1:6*p+3));
    count=count+1
end

%% 附加量：类似于燃料和能量消耗。









