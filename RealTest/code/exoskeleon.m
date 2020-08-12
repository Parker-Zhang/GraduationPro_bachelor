clear 
clc
% 变量声明
alpha =0; %前后伸展 正
beta =0;%沿颈肩上下伸展 负
gamma =0;% 负
coefficient = 10; %1表示单位 1m; 10 表示单位 dm ;100表示单位 cm ;1000表示单位mm

% 作为新坐标的原点
A0 = [1.832343 0.556926 1.438875]*coefficient;
C0 = [1.8342155;0.5480115;1.373666]*coefficient-A0';  %手臂末端坐标
L = norm(C0);
V2 = [1.704463 0.52516 1.596931]*coefficient-A0;
F = [1.766782 0.466348  1.537818]*coefficient-A0;
A =[(F+A0)/coefficient        ;
        1.742190 0.544417  1.538995;
        1.742190 0.546938  1.490152;
       1.891779 0.551402   1.591663;
       1.919727 0.567959  1.526999]*coefficient-kron(ones(5,1),A0);
%BV2 BV3 BV4 BV0 BV1 在坐标系O的表示
cB =[1.847434 0.536718 1.392759;
        1.8929344 0.524473  1.385093;
        1.819922 0.562993  1.369919;
        1.861352 0.550960  1.381767;
         1.850785 0.569201 1.366971]*coefficient-kron(ones(5,1),A0);
  
dalpha = 0;
dbeta = 0;
Tspan=1;
for k=1:1:Tspan
alpha = alpha +dalpha;
beta = beta+dbeta;
% 计算部分
cBT=cB';
AT=A';
% R = rpy2r(alpha,beta,gamma);
R = trotz(gamma)*troty(beta)*trotx(alpha);
C1 = homtrans(R,C0);%旋转顺序 Z，Y，X
T0=[ eye(3,3) ,zeros(3,1);zeros(1,3),1];
% T1表示中心的位姿 即 AcT
T1 = transl(C1')*trotz(gamma)*troty(beta)*trotx(alpha);
BT = homtrans(T1,cBT);
 
%  计算绳线的长度 A1B1
ML = AT-BT;
    for i=1:1:5
        stringL(k,i)=norm(ML(:,i));
    end
stringL
% 零位状态各绳线的长度为
% 1.8027    2.1635    1.4407    2.1209    1.7425    dm


%  画图部分
trplot(T0,'frame','O','color','r');
axis([-3*L+T0(1,end) 3*L+T0(1,end) -3*L+T0(2,end) 3*L+T0(2,end)  -3*L+T0(3,end) 3*L+T0(3,end) ]);
hold on
trplot(T1,'frame','C','color','b');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','r');
 plot3(V2(1),V2(2),V2(3),'o','color','b','MarkerSize',10,'MarkerFaceColor','r');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  进行连线 Ai 和Bi连线
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% 绘制Bi之间的连线
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(4,1)=0;
LinkB(1,4)=0;
gplot23D(LinkB,BT');
% 连接OC两点
OC =[zeros(3,1) C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
%连接 A1F两点
A1F=[V2' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end  
     
     
     
     