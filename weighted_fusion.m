function [spe_var,spe_D] = weighted_fusion(img,L)
%求不同分割尺度的分割权重
%   此处显示详细说明
[X,Y]=size(img);
DI=double(img(:));
L=L+1;
L_75=L;
num_L75=max(max(L_75));
Label75=double(L_75(:));
spe_var=zeros(X,Y);
mean_75=zeros(X,Y);
%计算光谱的方差
for i=1:num_L75
a(i)=var(DI(find(Label75==i)));
spe_var(find(L_75==i))=a(i);
end
%计算光谱的距离
for i=1:num_L75
mean_spe(i)=mean(DI(find(Label75==i)));
mean_75(find(L_75==i))=mean_spe(i);
end
spe_D=sqrt((double(img)-mean_75).^2);
% W=1./(var.*spe_D);
end

