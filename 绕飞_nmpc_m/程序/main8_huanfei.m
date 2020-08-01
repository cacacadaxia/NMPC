clc;
clear all;
warning('off');

%%
%Spacecraft initial state parameters
Mu=4.794e3;%��������
re =1750;%����뾶
c20=-0.113;
c22=0.0396;
% xit0=0;%��½��ྭ
% fai0=pi/2;%��½���γ
W =2*pi/5.27/3600;%С�����������ٶ�
% Gl=[-cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    cos(fai0);
%     -sin(fai0)               -cos(xit0)            0    ;
%     cos(xit0)*cos(fai0)  -sin(xit0)*cos(fai0)    sin(fai0)];
Gl=[1 0 0;0 1 0;0 0 1];
We=Gl'*[0;0;W];
Wex=We(1);
Wey=We(2);
Wez=We(3);

R0l =5830;%С�������ĵ���½��ľ���,�������ƽ����������
rol=[0;0;R0l];%С��������ָ����½�����������½����ϵLϵ������
AA=[Wex^2-W^2   Wex*Wey    Wex*Wez;
    Wex*Wey    Wey^2-W^2   Wey*Wez;
    Wex*Wez     Wey*Wez   Wez^2-W^2];%��˾���
BB=[   0    -2*Wez    2*Wey;
    2*Wez     0     -2*Wex;
    -2*Wey   2*Wex      0  ];%��˾���

global Mu re c20 c22


%% ������ʼ����
% r0=[-12507;-12000;-9500];
r0=[20000;-20000;0];
v0=[-1.0;0.1;-0.3];
rf=[1507;9870;-1500];%�ն�λ��


%%
%Spacecraft parameters and Time simulation parameters:
M=100;
T=4000;
zeta=2500;

p=40;%Ԥ������;��Ҫ������
m=40;%��������;

u_max=ones(3,1);
u_min=-ones(3,1);
%%  %ģ�����
omega=2*pi/5.27/3600;%С�����������ٶ�
ww=get_U(Gl*(r0+rol));%�����г����ϵ��
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


ccc=zeros(6,N);%�洢��
ddd=zeros(3,N);%�洢��
eee=zeros(6,N);%�洢��
fff=zeros(3,N);%�洢��
ggg=zeros(3,N);%�洢��
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
grid on;%��ά�Ļ���5%%%%%

for i=1:N
    R=zeros(3*p,1);%�õ������켣
    [rou,non]=hyperplanes(r0);%rou�Ǿ��뵱ǰλ�ó�ƽ��������ĵ㣬non�Ƿ�����
%     
%     if dot(r0-rou,rf-rou)<=0
%         rp=projection(non,rf,rou);%���һ���������õ��ն�λ��
%     else
%         rp=rf;
%     end
%     norm(rp-rf)
    
    for j=1:p
        tra=trajectory(i+j);%��������Ϊ0.05,��˼˵ѭ����ǧ����ʵ������200��
%         R(3*j+1:3*j+3,:)=tra(1:3);
%             R(3*j+1:3*j+3,:)=[6507;1000;9500];
            R(3*j+1:3*j+3,:)=rf;%�ı��ն�Լ��

    end
    tra=trajectory(i+p);

    

    %%
    cvx_begin quiet
        variable XX((p+1)*6+3*m,1) %˫����
        
        
        a=0;
        for q=1:m
%             a=a+norm(XX(6*(p+1)+3*q-2:6*(p+1)+3*q));%������Ӧ�ü�Ȩ�������ģ�����2����
                a=a+1*XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%����2������ƽ��
        end
        
        for r=1:p
%             a=a+norm(XX(6*r-5:6*r-3)-R(end-2:end));%���Լ�����ڼ����˷��ն�״̬���ն�λ������ļ�Ȩ
%                  a=a+1*(XX(6*r-5:6*r-3)-R(end-2:end))'*(XX(6*r-5:6*r-3)-R(end-2:end));%2������ƽ��
   %Ϊɶ���˷��ն�Լ��֮���Ż����ͺ����׵õ�nan�Ľ��أ�
        end
        c=1e-2;%�����ն�Լ���ļ�Ȩ
%         a=a+norm(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%�����ն�Լ����2����
            a=a+c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%2������ƽ��
            
        
    minimize(a);%��Ҫ�����ۺ�������ȼ�ϵļ�Ȩ�ټ����ն�λ�õļ�Ȩ
    subject to
        XX(1:6)==[r0;v0];
        for k=1:p
            XX(6*k+1:6*k+6)==A*XX(6*k-5:6*k)+B*XX(6*(p+1)+3*k-2:6*(p+1)+3*k);
            norm(XX(6*k+4:6*k+6))<=40;%�����ٶȵ�Լ��
            non'*(XX(6*k+1:6*k+3)-rou)>=0;%��ƽ��Լ��
        end
        
%      XX(6*(p+1)-5:6*(p+1)-3)==R(end-2:end);
        abs(XX(6*(p+1)+1:6*(p+1)+3*m))<=1*ones(3*m,1);
        
    cvx_end
    
    %% ���Ե���,���ӵ���������5���ٷֺ�
%     a%%%%%
%     a_without_u=c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))%%%%%
%     xf=XX(6*(p+1)-5:6*(p+1)-3)%%%%%
%     
%     a_safe=22000;
%     b_safe=10000;
%     c_safe=10000;
%     if xf(1)^2/a_safe^2+xf(2)^2/b_safe^2+xf(3)^2/c_safe^2>=0.99
%         fprintf('%s','�������Լ��');
%     else
%         fprintf('%s','���������Լ��');
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
    
%     ww=get_U(Gl*(r0+rol));%�����г����ϵ��
%     A  =[zeros(3) eye(3);
%         [omega^2+ww(1)  0  0;
%         0  omega^2+ww(2)  0;
%         0  0  ww(3)] [0 2*omega 0;
%         -2*omega 0 0;
%         0    0     0]];
%     [A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,1);
    
    
    
% ccc(1:3,i)= w_estimate;%��¼����
    ddd(:,i)=v0;%��¼����
    eee(:,i)=trajectory(i);%��¼����
% fff(:,i)=detu;%��¼����
    ggg(:,i)=uu;
    hhh(:,i)=[r0;v0]-trajectory(i);
    iii(:,i)=[r0;v0]+[0;0;0;0;0;0];
    fuel=fuel+norm(uu)*10;
    precision=precision+norm(XX(6*p+1:6*p+3));
    count=count+1
end

%% ��������������ȼ�Ϻ��������ġ�

fuel
precision

% plot_huanfei;







