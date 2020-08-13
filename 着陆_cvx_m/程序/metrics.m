


global N
%% fuel

fuel=0;
for i=1:N
   fuel=fuel+norm(ggg(:,i),1)*delta; 
end
fuel

%% precision
precision=norm(iii(1:3,end)-rf_l,2)%detrf

%%
% 之后的两个指标太虚了吧，有没有都一样啊。