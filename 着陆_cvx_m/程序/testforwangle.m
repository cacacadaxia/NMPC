

clear all;

% 两个坐标系，初始坐标系W系，新坐标系b系
W_T_wb = [100;200;300];     % 向量wb在W系下的坐标

x = W_T_wb(1);
y = W_T_wb(2);
z = W_T_wb(3);

% psi0                          %不考虑
xit0 = atan(y/x);               %roll
fai0 = atan( z/sqrt(x^2+y^2) ); %pitch

% xit0 = 1.4785;    % 经度
% fai0 = 0;         % 纬度
Rwb = [cos(xit0)*sin(fai0)   sin(xit0)*sin(fai0)    -cos(fai0);
        -sin(xit0)               cos(xit0)            0    ;
        cos(xit0)*cos(fai0)   sin(xit0)*cos(fai0)    sin(fai0)]';%Cal

% 验证：Rwb*[0;0;1]：
% [0;0;1]在b系下
% Rwb*[0;0;1]表示在W系下
%  Rwb*[0;0;1]与W_T_wb方向一致
% 所以应该没啥问题
W_T_wb./(Rwb*[0;0;1])






