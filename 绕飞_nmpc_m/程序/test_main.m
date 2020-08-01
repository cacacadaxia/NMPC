function test_main
% warning off
%% dynamic û���õ�����
xit0=1.4785;%����
fai0=0;
Gl=[cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    -cos(fai0);
    -sin(xit0)               cos(xit0)            0    ;
    cos(xit0)*cos(fai0)   sin(xit0)*cos(fai0)    sin(fai0)]';
omega=2*pi/5.27/3600;%С�����������ٶ�
% r0=[1500;-590.913844154563;4543.78365683008];%��½����ϵ
   r0    =    [20000;-20000;0];
   v0    =    [-1.0;0.1;-0.3];
   rf    =    [1507;9870;-1500];%�ն�λ��
   xf    =    [rf;zeros(3,1)];
   R0l   =    5423.1;%С�������ĵ���½��ľ���
   rol   =    [0;0;R0l];%��½����ϵ
   ww    =    Gl'*get_U(Gl*(r0+rol));%�����г����ϵ��
   Ac  =[zeros(3) eye(3);
    [omega^2+ww(1)  0  0;
    0  omega^2+ww(2)  0;
    0  0  ww(3)                ] [0 2*omega 0;
    -2*omega 0 0;
    0    0     0]];
Bc=[zeros(3);eye(3)];
Cc=[eye(3) zeros(3)];
Dc=zeros(3);
     T            = 10;%�ȷ�0.1�⿴��
     [A,B,C,D]    = c2dm(Ac,Bc,Cc,Dc,T);
global A B xf
%% �����˾�������
     P            =diag([15,25,20,25,15,45]);%blkdiag�ն�
     Q            =1e-4*blkdiag(eye(3),50*eye(3));
     R            =1*eye(3);%����
     N            =15;
   type           = 'difference equation';
atol_ode_real     = 1e-12;%ò��ûʲô����
rtol_ode_real     = 1e-12;
atol_ode_sim      = 1e-4;
rtol_ode_sim      = 1e-4;
  opt_option      = 0;
    tol_opt       = 1e-8;
    u0            = 0.2*ones(3,N);
 xmeasure         = [r0;v0]';
 tmeasure         = 0.0;
 mpciterations    = 300;
 

global P Q R
%% bug
[t,x,u]=nmpc(@runningcosts, @terminalcosts, @constraints, ...
    @terminalconstraints, @linearconstraints, @system, ...
    mpciterations, N, T, tmeasure, xmeasure, u0, ...
    tol_opt, opt_option, ...
    type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim);

u=reshape(u,3,size(u,1)/3);
plot_pictures(t,x,u);

save('data_no_v','u','t','x');


end


function cost=runningcosts(t,x,u)
global Q R xf
cost=(x-xf')*Q*(x-xf')'+u'*R*u;
% cost=x(1);
end

function cost = terminalcosts(t, x)
global P xf
cost = (x-xf')*P*(x-xf')';
% cost=0;
end

function [c,ceq] = constraints(t, x, u)%��Լ��

% xf(1)^2/a_safe^2+xf(2)^2/b_safe^2+xf(3)^2/c_safe^2>=0.99;
% ���ﵽ������ʲô����ϵ�µģ�ò�ƾ�����С�����������ϵ�µġ�

vmax=20;
vmin=-20;
a_safe=22000;
b_safe=10000;
c_safe=10000;
c(1) = x(4)-vmax;
c(2) = x(5)-vmax;
c(3) = x(6)-vmax;
c(4) = -x(4)+vmin;
c(5) = -x(5)+vmin;
c(6) = -x(6)+vmin;
c(7) = 0.99-(x(1)^2/a_safe^2+x(2)^2/b_safe^2+x(3)^2/c_safe^2);
%Ӧ��û��ô�򵥰ɡ�
c=[];
ceq = [];
end


function [c,ceq] = terminalconstraints(t, x)%���ն�Լ��
c   = [];%�ն�Լ��ֵ���״̬����
ceq = [];


end
function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
A   = [];%ֻ������Լ��ô��
b   = [];
Aeq = [];
beq = [];
lb  = -0.4;
ub  = 0.4;
end

function y = system(t, x, u, T)
global A B
y =A*x'+B*u;
y=y';
% y(1:3)=u(1:3);
% y(4:6)=zeros(3,1);

end

function plot_pictures(t,x,u)

figure(1)
plot(t,x(:,1),t,x(:,2),t,x(:,3));
grid on
title('r');

figure(2)
plot(t,x(:,4),t,x(:,5),t,x(:,6));
grid on
title('v');

figure(3)
plot(t,u(1,:),t,u(2,:),t,u(3,:))
grid on;
title('u');

end
