function [rou,non]=hyperplanes(r)


warning('off');
a=22000;
b=10000;
c=10000;
W=[1/a^2 0 0;0 1/b^2 0;0 0 1/c^2];
global a b c;

f=@(x,y,z)fzero(@(lamda)a^2*x^2/(lamda+a^2)^2+b^2*y^2/(lamda+b^2)^2+c^2*z^2/(lamda+c^2)^2-1,0);

lamda=f(r(1),r(2),r(3));

rou = inv(eye(3)+lamda*W)*r;

non=(r-rou)/norm(r-rou);


end