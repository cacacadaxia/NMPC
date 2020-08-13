global N;
l=(1:N)*2;

figure;
plot(l,ddd(1,:),l,ddd(2,:),l,ddd(3,:));
grid on;
title('速度变化');

rrr=[];
for i=1:N
   rrr=[rrr norm(ddd(:,i))]; 
end
c33([l' l'],[rrr' 10*ones(1,N)'])


fuel=0;
for i=1:N
   fuel=fuel+2*norm(ggg(:,i),1); 
end
fuel

mm=[0];
for i=2:N
   mm=[mm mm(i-1)+2*norm(ggg(:,i),1)]; 
end
figure;
plot(l,mm)
grid on;
xlabel('仿真时间 t(s)')
ylabel('燃耗指数变化 m(kg)')
c33(l,mm)


% figure;
% plot(l,eee(4,:),l,eee(5,:),l,eee(6,:));
% grid on;
% title('期望速度变化');


% figure;
% plot(l,hhh(1,:),l,hhh(2,:),l,hhh(3,:));
% grid on;
% % axis square;
% title('与轨迹之间的位置误差');


% figure;
% plot(l,hhh(4,:),l,hhh(5,:),l,hhh(6,:));
% grid on;
% % axis square;
% title('与轨迹之间的速度误差');


cu(l,[ggg(1,:);0.15*ones(size(l));-0.15*ones(size(l))],[ggg(2,:);0.15*ones(size(l));-0.15*ones(size(l))],[ggg(3,:);0.15*ones(size(l));-0.15*ones(size(l))])
% c22(l,iii(1:3,:))
c11(l,iii(4:6,:))
% % c22(l,iii(1:3,:))

% figure;
% plot(l,ccc(1,:),l,ccc(2,:),l,ccc(3,:));
% grid on;
% title('input observer')


figure;%画出轨迹相关的图
load Eros433;
h=patch('vertices',vertex,'faces',facet,'facecolor',[1 1 1],'edgecolor',[0 0 0]);
% alpha(.33);
hold on
% ellipsoid(0,0,0,22000,10000,10000,20);
 alpha(.33);
% for k=1:N
%     hold on;
%     plot3(iii(1,k),iii(2,k),iii(3,k),'.');
% end
hold on;
% w=[6507;1000;9500];
% plot(w,'p');
plot3(jjj(1,:),jjj(2,:),jjj(3,:),'r');
f1=plot3(jjj(1,1),jjj(2,1),jjj(3,1),'p');
f2=plot3(jjj(1,N),jjj(2,N),jjj(3,N),'p');
grid on;%三维的画线
title('输出状态');
 legend([f1,f2],'data4','data2')

 hold on
 nn=10000
w=zeros(3,nn);
Nn=[3;3;0.5];
for i=1:nn
    w(:,i)=i*Nn+[500;5400;0];
end
plot3(w(1,:),w(2,:),w(3,:))



% figure;
% plot3(iii(1,:),iii(2,:),iii(3,:),'r');
% grid on;%三维的画线5


figure;
plot(l,iii(1,:),l,iii(2,:),l,iii(3,:));
grid on;
title('输出状态变化曲线');
hold on
% plot(N,6507,'p',N,1000,'p',N,9500,'p')

%% test

% figure;
% plot(l,-ccc(1,:),l,ggg(1,:));
% grid on;
% title('对比');









