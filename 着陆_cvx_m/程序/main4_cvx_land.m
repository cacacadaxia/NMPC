clc;
clear all;
warning('off');
%% some bullshit
Mu=4.794e3;%��������
re =1750;%����뾶
c20=-0.113;
c22=0.0396;
xit0 = 1.4785;    % ����
fai0 = 0;         % γ��
W =2*pi/5.27/3600;%С�����������ٶ�
Gl=[cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    -cos(fai0);
    -sin(xit0)               cos(xit0)            0    ;
    cos(xit0)*cos(fai0)   sin(xit0)*cos(fai0)    sin(fai0)]';%Cal
We=Gl'*[0;0;W];Wex=We(1);Wey=We(2);Wez=We(3);
R0l =5423.1;%С�������ĵ���½��ľ���
rol=[0;0;R0l];%��½����ϵ
AA=[Wex^2-W^2   Wex*Wey    Wex*Wez;
    Wex*Wey    Wey^2-W^2   Wey*Wez;
    Wex*Wez     Wey*Wez   Wez^2-W^2];%��˾���
BB=[   0    -2*Wez    2*Wey;
    2*Wez     0     -2*Wex;
    -2*Wey   2*Wex      0  ];%��˾���

global Mu re c20 c22
%%
%% ������ʼ����
% r0=[1507;9870;-1500];%��С�����������ϵ
r0=[1500;-590.913844154563;4543.78365683008];%��½����ϵ
v0=[-1.0;0.1;-0.3];
%�ն�
%rrf=[500;5400;0];%С�����������ϵ
%rrf=[0;0;0];%��½����ϵ,��½���Ǵ�r0��rff�Ĺ��̡�

%%
p=20;%Ԥ������;��Ҫ������
m=p;%��������;
delta=2;%����
N=400;
global N

%% ģ�����  ���ò�ַ���Ԥ�⶯��ѧ
omega=2*pi/5.27/3600;%С�����������ٶ�
ww=Gl'*get_U(Gl*(r0+rol));%�����г����ϵ��
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

%% ����Լ��
Nn=[3;3;0.5];%С�����������ϵ��
xit1=0.7854;%������������ģ���
fai1=0.1173;
Gn=[cos(xit1)*sin(fai1)   sin(xit1)*sin(fai1)    -cos(fai1);
    -sin(xit1)               cos(xit1)            0    ;
    cos(xit1)*cos(fai1)   sin(xit1)*cos(fai1)    sin(fai1)]';%GС�����������ϵ��nϵ��������ϵ
Cln=Gl'*Gn;
rf_l=[0;0;0];%����½����ϵ�µ��ն�λ�á�
dp=150;%��ȫ����
ap=400;%��������������Լ��������
% rp=rf_l-dp*(Nn/norm(Nn));
rp=rf_l-dp*(Cln*[0;0;1]);%��½ϵ��
%% lϵ����½����ϵ��nϵ�Ǳ�������ϵ
%% ���������õĹ�������ϵ�������Ҿ�������½����ϵò�Ƹ��Ӻ���

%% input observer
%%  %main%
count=0;
fuel=0;
precision=0;
for i=1:N
    R=zeros(3*(p+1),1);%�õ�����
    
    PP=diag([15,25,20,25,15,45]);%blkdiag�ն�
    %     QQ=diag([15,25,20,25,15,45]);%״̬
    QQ=1e-2*PP;
    %     QQ=zeros(6);
    RR=10*eye(3);%����
    cvx_begin quiet
    variable XX((p+1)*6+3*m,1) %˫����
    a=0;
    %% ��������Ĳ���
    PP=1e-3*blkdiag(diag([1,2,5]),2*eye(3));
    QQ=0.001*PP;
    RR=10*eye(3);
    for q=1:m
        a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*RR*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%����2������ƽ��
    end
    for r=1:4
        a=a+(XX(6*r-5:6*r)-R(end-5:end))'*QQ*(XX(6*r-5:6*r)-R(end-5:end));%2������ƽ��������״̬���ۺ���
    end
    a=a+(XX(6*(p+1)-5:6*(p+1))-R(end-5:end))'*PP*(XX(6*(p+1)-5:6*(p+1))-R(end-5:end));%2������ƽ��
    %% 2017��Ĳ���
    %         for q=1:m
    %             a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*RR*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%����2������ƽ��
    %         end
    %         for r=1:p
    %             a=a+(XX(6*r-5:6*r)-R(end-5:end))'*QQ*(XX(6*r-5:6*r)-R(end-5:end));%2������ƽ��������״̬���ۺ���
    %         end
    %         a=a+(XX(6*(p+1)-5:6*(p+1))-R(end-5:end))'*PP*(XX(6*(p+1)-5:6*(p+1))-R(end-5:end));%2������ƽ��
    %% ֮ǰ�����õ�һЩ���ۺ���
    %         for q=1:m
    % %             a=a+norm(XX(6*(p+1)+3*q-2:6*(p+1)+3*q));%������Ӧ�ü�Ȩ�������ģ�����2����
    %                 a=a+XX(6*(p+1)+3*q-2:6*(p+1)+3*q)'*XX(6*(p+1)+3*q-2:6*(p+1)+3*q);%����2������ƽ��
    %         end
    %         for r=1:p
    % %             a=a+norm(XX(6*r-5:6*r-3)-R(end-2:end));%���Լ�����ڼ����˷��ն�״̬���ն�λ������ļ�Ȩ
    % %                  a=a+0.01*(XX(6*r-5:6*r-3)-R(end-2:end))'*(XX(6*r-5:6*r-3)-R(end-2:end));%2������ƽ��
    %    %Ϊɶ���˷��ն�Լ��֮���Ż����ͺ����׵õ�nan�Ľ��أ�
    %         end
    %
    %         c=1e-4;
    % %         a=a+norm(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%�����ն�Լ����2����
    %         a=a+c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end));%2������ƽ��
    %%
    minimize(a);%��Ҫ�����ۺ�������ȼ�ϵļ�Ȩ�ټ����ն�λ�õļ�Ȩ������漰������ļ�Ȩ�����ֵ�Ƕ��٣�
    subject to
    XX(1:6)==[r0;v0];
    for k=1:p
        XX(6*k+1:6*k+6)==A*XX(6*k-5:6*k)+B*XX(6*(p+1)+3*k-2:6*(p+1)+3*k);
        norm(XX(6*k+4:6*k+6))<=10;%�����ٶȵ�Լ��
        (XX(6*k+1:6*k+3)-rp)'*Cln*[1/ap^2  0  0;0  1/ap^2  0;0  0  0]*Cln'*(XX(6*k+1:6*k+3)-rp)-[0;0;1]'*Cln'*(XX(6*k+1:6*k+3)-rp)<=0
        %������Լ��
    end
    
    for k=1:4
        [0;0;1]'*XX(6*k+1:6*k+3)>=0;
    end
    %      XX(6*(p+1)-5:6*(p+1)-3)==R(end-2:end);
    abs(XX(6*(p+1)+1:6*(p+1)+3*m))<=0.15*ones(3*m,1);
    cvx_end
    
    %% ���Ե���,���ӵ���������5���ٷֺ�
    %     a%%%%%
    %     a_without_u=c*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))'*(XX(6*(p+1)-5:6*(p+1)-3)-R(end-2:end))%%%%%
    %     xf=XX(6*(p+1)-5:6*(p+1)-3)%%%%%
    %     if (r0-rp)'*Cln*[1/ap^2  0  0;0  1/ap^2  0;0  0  0]*Cln'*(r0-rp)-[0;0;1]'*Cln'*(r0-rp)<=0
    %         fprintf('����Լ��\n')
    %     else
    %         fprintf('shit mother fuck\n')
    %     end
    %%
    uu=XX(6*(p+1)+1:6*(p+1)+3)
    
    sim('dynamic_for_landing1',delta);
    RV_result=simout(end,:)';%dynamic
    r0=RV_result(1:3)%+0.4*randn(3,1)
    v0=RV_result(4:6)%+0.03*randn(3,1);
    
    %     ww=Gl'*get_U(Gl*(r0+rol));%�����г����ϵ��
    %     Ac  =[zeros(3) eye(3);
    %         [omega^2+ww(1)  0  0;
    %         0  omega^2+ww(2)  0;
    %         0  0  ww(3)] [0 2*omega 0;
    %         -2*omega 0 0;
    %         0    0     0]];
    %     [A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,delta);
    
    ddd(:,i)=v0;%��¼����
    ggg(:,i)=uu;
    iii(:,i)=[r0;v0];
    jjj(1:3,i)=Gl*(iii(1:3,i)+rol);
    fuel=fuel+norm(uu,1)*delta;
    precision=precision+norm(XX(6*p+1:6*p+3));
    count=count+1
end

%% ��������������ȼ�Ϻ��������ġ�









