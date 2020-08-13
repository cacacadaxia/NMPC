


w=zeros(3,1000);
Nn=[3;3;0.5];
for i=1:1000
    w(:,i)=[500;5400;0]+i*Nn;
end


figure;
plot3(w(1,:),w(2,:),w(3,:))