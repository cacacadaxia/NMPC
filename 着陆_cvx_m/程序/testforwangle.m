

clear all;

% ��������ϵ����ʼ����ϵWϵ��������ϵbϵ
W_T_wb = [100;200;300];     % ����wb��Wϵ�µ�����

x = W_T_wb(1);
y = W_T_wb(2);
z = W_T_wb(3);

% psi0                          %������
xit0 = atan(y/x);               %roll
fai0 = atan( z/sqrt(x^2+y^2) ); %pitch

% xit0 = 1.4785;    % ����
% fai0 = 0;         % γ��
Rwb = [cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    -cos(fai0);
        -sin(xit0)               cos(xit0)            0    ;
        cos(xit0)*cos(fai0)   sin(xit0)*cos(fai0)    sin(fai0)]';%Cal

% ��֤��Rwb*[0;0;1]��
% [0;0;1]��bϵ��
% Rwb*[0;0;1]��ʾ��Wϵ��
%  Rwb*[0;0;1]��W_T_wb����һ��
% ����Ӧ��ûɶ����
W_T_wb./(Rwb*[0;0;1])






