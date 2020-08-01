function tra= trajectory(t)
%X=[x;y;z]
%%
x0=100;
y0=-200;
z0=1000;
dot_x0=-1.0;
dot_y0=0.1;
dot_z0=-0.3;
xf=0;
yf=0;
zf=0;
dot_xf=0;
dot_yf=0;
dot_zf=0;

T=200;
zeta=200;
%%

if t<=zeta
    x=x0+dot_x0*t+(3*xf-3*x0-2*dot_x0*zeta)*(t/zeta)^2+(2*x0+dot_x0*zeta-2*xf)*(t/zeta)^3;
    dot_x=dot_x0+2*(3*xf-3*x0-2*dot_x0*zeta)*t/zeta^2+3*(2*x0+dot_x0*zeta-2*xf)*t^2/zeta^3;
    y=y0+dot_y0*t+(3*yf-3*y0-2*dot_y0*zeta)*(t/zeta)^2+(2*y0+dot_y0*zeta-2*yf)*(t/zeta)^3;
    dot_y=dot_y0+2*(3*yf-3*y0-2*dot_y0*zeta)*t/zeta^2+3*(2*y0+dot_y0*zeta-2*yf)*t^2/zeta^3;
    
elseif t>zeta
    x=0;
    dot_x=0;
    y=0;
    dot_y=0;
end
if t<=T
    z=z0+dot_z0*t+(3*zf-3*z0-2*dot_z0*T)*(t/T)^2+(2*z0+dot_z0*T-2*zf)*(t/T)^3;
    dot_z=dot_z0+2*(3*zf-3*z0-2*dot_z0*T)*t/T^2+3*(2*z0+dot_z0*T-2*zf)*t^2/T^3;
elseif t>zeta
    z=0;
    dot_z=0;
end
tra=[x;y;z;dot_x;dot_y;dot_z];
%%
% 
% af=zeros(3,1);
% v0=[dot_x0;dot_y0;dot_z0];
% vf=zeros(3,1);
% r0=[x0;y0;z0];
% rf=[0;0;0];
% 
% C0=af-6*()




