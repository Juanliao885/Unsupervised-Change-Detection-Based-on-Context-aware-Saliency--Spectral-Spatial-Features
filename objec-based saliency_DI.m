I1=imread('CubbieStation_im1.jpg');
I2=imread('CubbieStation_im2.jpg');
L=imread('CubbieStation_50.tif');L=L+1;
img_d=sqrt((double(I2(:,:,1))-double(I1(:,:,1))).^2+(double(I2(:,:,2))-double(I1(:,:,2))).^2+(double(I2(:,:,3))-double(I1(:,:,3))).^2);
ymax=255;ymin=0;
xmax = max(max(img_d)); 
xmin = min(min(img_d)); 
out_img_d= round((ymax-ymin)*(img_d-xmin)/(xmax-xmin) + ymin); 
%img = ind2rgb(gray2ind(uint8(Outimg_d),255),jet(255));灰度转换为伪彩色。
%[ segs ] = Msseg(img,11,13,100);% [ segs ] = Msseg( img,hs,hr,M ); hs:空间带宽  hr:范围域带宽  M: 最小超像素所包含的像素个数 
%Mimg=zeros(X*Y,Z);
%img1=reshape(I1,X*Y,Z);
%for i=1:size(segs.seg,2)
   % for j=1:Z
       %Mimg(segs.seg{i},j) = round(mean(img1(segs.seg{i},j)));
    %end
%end
%seg_img1 = reshape(Mimg,[X,Y,Z]);
%different image: img=sqrt((segs.seg_img(:,:,1)-seg_img1(:,:,1)).^2+(segs.seg_img(:,:,2)-seg_img1(:,:,2)).^2+(segs.seg_img(:,:,3)-seg_img1(:,:,3)).^2); 
img(:,:,1)=out_img_d;img(:,:,2)=out_img_d;img(:,:,3)=out_img_d;
[X,Y,Z] = size(img);
numlabels=max(max(L));
nseg = max(L(:));
for i=1:nseg
    sum_x=0;sum_y=0;area=0;
    for m=1:X
      for n=1:Y
          if L(m,n)==i
            sum_x=sum_x+m;
            sum_y=sum_y+n;
            area=area+1; 
            end
        end
    end
     plot(i,1)=fix(sum_x/area);
     plot(i,2)=fix(sum_y/area);
end
Img{1}=img(:,:,1);Img{2}=img(:,:,2);Img{3}=img(:,:,3);
for i=1:nseg
  ND_spe(i,1)=mean(Img{1}(find(L==i)));
  ND_spe(i,2)=mean(Img{2}(find(L==i)));
  ND_spe(i,3)=mean(Img{3}(find(L==i)));
end
vals = reshape(img,X*Y,Z);
[points edges]=lattice(X,Y,0);    clear points;
d_edges = edges(find(L(edges(:,1))~=L(edges(:,2))),:);
all_seg_edges = [L(d_edges(:,1)) L(d_edges(:,2))]; all_seg_edges = sort(all_seg_edges,2);
tmp = zeros(nseg,nseg);
tmp(nseg*(all_seg_edges(:,1)-1)+all_seg_edges(:,2)) = 1;
[edges_x edges_y] = find(tmp==1); seg_edges = [edges_x edges_y];
L1=seg_edges;
for i=1:nseg
a=seg_edges(L1(:,1)==i|L1(:,2)==i,:);
a=a(:)';
a(find(a==i))=[];
A{i}{1}=a;
A{i}{2}=[];
k1=length(A{i}{1});
for j=1:k1
   a=seg_edges(L1(:,1)==A{i}{1}(j)|L1(:,2)==A{i}{1}(j),:);
   a=a(:)';
   a(find(a==i))=[];
   A{i}{2}=unique([A{i}{2} a]);
end
    k2=length(A{i}{2}(:));
    A{i}{3}=[];
 for v=1:k2
   a=seg_edges(L1(:,1)==A{i}{2}(v)|L1(:,2)==A{i}{2}(v),:);
   a=a(:)';
   a(find(a==i))=[];
   A{i}{3}=unique([A{i}{3} a]);
 end
   k3=length(A{i}{3}(:));
    A{i}{4}=[];
    for w=1:k3
   a=seg_edges(L1(:,1)==A{i}{3}(w)|L1(:,2)==A{i}{3}(w),:);
   a=a(:)';
   a(find(a==i))=[];
   A{i}{4}=unique([A{i}{4} a]);
 end
k4=length(A{i}{4}(:));
    A{i}{5}=[];
    for r=1:k4
   a=seg_edges(L1(:,1)==A{i}{4}(r)|L1(:,2)==A{i}{4}(r),:);
   a=a(:)';
   a(find(a==i))=[];
   A{i}{5}=unique([A{i}{5} a]);
    end
end   
k=3;                                                                        
ND_diss=[];
for p=1:nseg
e=A{p}{k};
NDsur_pixel=[];
for q=1:length(e)
   D=ND_spe(e(q),:);
   NDsur_pixel=[NDsur_pixel;D];
 end
   nd_sp=repmat(ND_spe(p,:),length(e),1);%p
   diss=NDsur_pixel-nd_sp;
   nd_diss=sum(sqrt(sum(diss.^2')))/length(e);
   clear e;
   ND_diss=[ND_diss;nd_diss];
end
ymax=1;ymin=0;
DNxmax = max(ND_diss);DNxmin = min(ND_diss);
ND_diss=(ymax-ymin)*(ND_diss-DNxmin)/(DNxmax-DNxmin) + ymin;%归一化到【0,1】
SP_diss=[];
for h=1:nseg
e=A{h}{k};
SPsur_pixel=[];
for g=1:length(e)
   SP=plot(e(g),:);
   SPsur_pixel=[SPsur_pixel;SP];
end
 nd_SP=repmat(plot(h,:),length(e),1);
 diss_SP=SPsur_pixel-nd_SP;
 sp_diss=sum(sqrt(sum(diss_SP.^2')))/length(e);
    clear e;
   SP_diss=[SP_diss;sp_diss];
end
SPxmax = max(SP_diss);SPxmin = min(SP_diss);
SP_diss=(ymax-ymin)*(SP_diss-SPxmin)/(SPxmax-SPxmin) + ymin;%归一化到【0,1】
C=3;                                                             
DISS=ND_diss./(1+C.*SP_diss);
L_=L(:);
for x=1:X*Y
    img_ND_diss(x)=DISS(L_(x));
end  
img_ND_diss=reshape(img_ND_diss,X,Y);
imshow(img_ND_diss,[]);
