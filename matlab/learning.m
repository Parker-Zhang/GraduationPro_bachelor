%% ͼ�����·��
clear all
A=[0 1 0 2 0 0 ;
      0 0 3 4 0 0 ;
      0 -2 0 5 1 0;
      0 4 0 0 -3 0;
      0 0 2 3 0 0;
      0 0 2 0 2 0];
  G=digraph(A,{'V1','V2','V3','V4','V5','V6'});
  TR=shortestpathtree(G,1);
  p = plot(G,'EdgeLabel',G.Edges.Weight);
  highlight(p,TR,'EdgeColor','r');
%%   inputѧϰ
clear all
C = input('Enter a num\n');
F = (C*1.8)+32;
fprintf('�����¶�=%.2f\n',F);
%% ����ɢ��ͼͼ
clc
clear all
% ������ͼ �����꣬��˳�� xlabel ylabel title color LineStyle legend��ͼ����
% hold on hold off figure���������ڣ�subplot�������ڻ��֣�
% ����ɢ��ͼ
N=100;
x=2*rand(1,N)-1;%1 �� N ��
y=2*rand(1,N)-1;
% scatter(x,y,'filled');%'filled' ��ʾ���
in = x.^2 +y.^2 <= 1;
out = ~in;
hold on
% scatter(x(in),y(in));
% scatter(x(out),y(out),'x');
plot(x(in),y(in),'linestyle','none','marker','o');
plot(x(out),y(out),'linestyle','none','marker','x');
theta =0:0.01:2*pi;
circle_x = cos(theta);
circle_y = sin(theta);
plot(circle_x,circle_y,'color','black','linewidth',2);
%% ���Ʊ�ͼ
clc
clear all
%  ��������
labels = {'����','����','����','����'};
% ���Ĵ�С
X =[50 100 150 200];
% ��һ���Ҫ�������
explode = [0 0 0 1];
% ����ͼ
pie(X,explode,labels);
title('����ҵ��');
%% ͼ�ξ��
t=0:0.01:10;
y=sin(t);
hd=plot(t,y);%�õ������ߵľ��
% ͨ��set���������޸�ͼ�ε�����
h = gcf ;%�õ���ǰͼ��ľ��
%% ��������
clear
clc
t = -pi:0.01:pi;
h = plot(0,0);
 axis([-4,4,-1,1]);
%  ÿ��1/25��ˢ��һ������
fs = 1/25;
% ����һ��ʹ��pause���������Ƕ�����ʱ���ܲ���
% for i = 1:5:length(t)
%      set(h, 'xdata' , t(1:i), 'ydata', sin(t(1:i)));%���¾��������
%      pause(fs);%�����ڼ䲻�ܽ�����������
% end

%�����������ö�ʱ��
global i
i=1;
timer1=timer('Period',fs,'TimerFcn',{@callback,h,t},'ExecutionMode','fixedSpacing');
start(timer1);
%% ���ؿ��巨��PI����
clear
clc
R=10;
N=10;
fs = 0.8;

global X Y num
X=[];
Y=[];
num =0;
t=0:0.01:2*pi;
circle_x = R*cos(t);
circle_y = R*sin(t);
plot(circle_x,circle_y,'color','k');
axis('square');
timer1=timer('Period',fs,'TimerFcn',{@callback,R,N},'ExecutionMode','fixedSpacing');
hold on
start(timer1);
%% �����˹����䣻����ѵ��
clear
clc
% �Ȼ�һ����ԭ�������ϵ
T1 = SE2(1,2,30*pi/180);
axis([0 5 0 5]);
axis square;
hold on;
grid on;
 trplot2(T1,'frame','1','color','b');
 
 T2 = SE2(2,1,0);
trplot2(T2,'frame','2','color','r');

% ��� �� �ҳ˵�����
T3 = T1*T2; %�Ƚ���T1�任���ٽ���T2�任
trplot2(T3,'frame','3','color','g');

T4 = T2*T1; %�Ƚ���T2�任���ٽ���T1�任
trplot2(T4,'frame','4','color','c');

P = [3;2];
plot_point(P,'*');
P1 = double(inv(T1)) * [P; 1]; %�õ�P������ϵ1��λ��
h2e (P1);%h��ʾ�����ʽ��e��ʾŷ����õ� e2h
%  homtrans(double(inv(T1)),P); %���Ϊ��α任�ĺ��� �����ı��ʽ

%  ��ά�ռ�����
 R = rotx(pi/2);%��ʾ��x����תpi/2
 trplot(R);
 tranimate(R);%������ת����,����������ϵ��ת��ָ������ϵ����
 R = rotx(pi/2)*roty(pi/2);%��������ϵ����x��ת��90������y��ת��90�㣻
 
%  ŷ���Ǳ�ʾ���� ŷ���Ǳ�ʾ����һ���ض�������ת���Σ��������ظ���ת
% ZYZʽ
R = rotz(0.1)*roty(0.2)*rotz(0.3);
R = eul2r(0.1,0.2,0.3);%��ʽ���ʽ�Ľ����ͬ
% ������⣺�ҵ�������ת�����ŷ����,�������Ĺ�ϵ
gamma = tr2eul(R);
% �м��з��⣬��������ֵ�����⣬�һ�û�����
% ����㣻��ת���໥ƽ��
% ��Ҫ��ϸ��

% RPY�� �������ǣ����ں��պͳ������ԣ�x��Ϊǰ������z����ֱ���£�y��ָ�����ַ���
% Roll:��� Pitch: ���� Yaw: ƫ��(����) 
R = rpy2r(0.1,0.2,0.3);
gamma = tr2rpy(R);
%% ������沿�� 2D ����Ƕȣ�ȷ���˳�
% �Ȼ�һ������ϵ{O}
clear 
clc
T0 =SE2(0,0,0);
trplot(T0,'frame','O');
hold on;
grid on;
% ȷ������ϵ C
theta =-pi/6;
L =3;
axis([-(L+2) L -(L+2) L]);
% T = SE2(0,0,pi/2);
C = [0 ;-L ];
B = [1/3*C,2/3*C];%�ҵ�B������Ӧ�����C����ϵ���Ե�
A =[-1 1;1 -1];
R = double(SE2(0,0,theta));
C2 =homtrans(R,C);
B2 =homtrans(R,B);
Tc=SE2(C2(1),C2(2),theta);


trplot(Tc,'frame','C');
plot_point(B2,'*');
plot_point(C2,'*');
plot_point(A,'o');

P =[[0;0] B2  C2 A];
point_n=length(P);
Link=zeros(point_n,point_n);
for i=1:1:point_n/2
    Link(i,point_n/2+i)=1;
    Link(point_n/2+i,i)=1;
end
gplot(Link,P');
% line([A(1,1) B2(1,1)],[A(2,1) B2(2,1)]);
% line([A(1,2) B2(1,2)],[A(2,2) B2(2,2)]);
% line([0 C2(1,1)],[0 C2(2,1)]);

hold off;
ML =B2-A;
L1=norm(ML(:,1))
L2=norm(ML(:,2))
%% 3D ���� ����3ά����Ҫ����һ��������
% ��û����λ �����޷����������ռ� һ��alpha beta gamma����
% ����ʵ���ܹ�����Ŀռ�
clear 
clc
% ��������
alpha =0; %ǰ����չ ��
beta =pi/4;%�ؾ���������չ ��
gamma =0;% ��
L = 10;
H =8;
C0 = [0;0;-L];
A1 = [0 5 H];
F = [-10 0 H-3];
%F A2 A3 A4 A5...������ϵO�ı�ʾ
A =[F        ;
        0 5 H;
        0 5 H;
        0 -5 H;
        0 -5 H];
 %B1 B2 B3 B4 B5 ������ϵC�ı�ʾ
cB = [2 3 -2;
          1 2 2;
          2 -2 -2;
          1 -2 2;
          -2 0 -2 ];
dalpha = pi/200;
for k=1:1:50
alpha = alpha +dalpha;
% ���㲿��
cBT=cB';
AT=A';
% R = rpy2r(alpha,beta,gamma);
R = trotz(gamma)*troty(beta)*trotx(alpha);
C1 = homtrans(R,C0);%��ת˳�� Z��Y��X
T0=[ eye(3,3) ,zeros(3,1);zeros(1,3),1];
% T1��ʾ���ĵ�λ�� �� AcT
T1 = transl(C1')*trotz(gamma)*troty(beta)*trotx(alpha);
BT = homtrans(T1,cBT);
 
%  �������ߵĳ��� A1B1
ML = AT-BT;
stringL=zeros(1,5);
    for i=1:1:5
        stringL(1,i)=norm(ML(:,i));
    end
stringL
%  ��ͼ����
trplot(T0,'frame','O','color','r');
axis([-1.2*L 1.2*L -1.2*L 1.2*L  -1.2*L 1.2*L ]);
hold on
trplot(T1,'frame','C','color','b');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  �������� Ai ��Bi����
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% ����Bi֮�������
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(4,1)=0;
LinkB(1,4)=0;
gplot23D(LinkB,BT');
% ����OC����
OC =[[0;0;0] C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
%���� A1F����
A1F=[A1' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end 
% % ��ȡ����T�е���ת����
% R2=t2r(T1);
% % ��ȡ����T�е�ƽ�Ʋ���
% trans1(T)'
% % ����õ���̬�� ; NaN not a num 
% gamma=tr2rpy(R2);

%% ���ƹ켣����
% ��û����λ �����޷����������ռ� һ��alpha beta gamma����
clear
clc
% ��������
alpha =0; %ǰ����չ ��
beta =pi/4;%�ؾ���������չ ��
% beta = 0;
gamma =0;% ��
L = 10;
H =8;
% L = 200;
C0 = [0;0;-L];
A1 = [0 5 H];
F = [-10 0 H-3];

%F A2 A3 A4 A5...������ϵO�ı�ʾ A1 A4 A5 һ�� A2 A3һ�� 
A =[F        ;
        0 -5 H;
        0 -5 H;
        0 5 H;
        0 5 H];
% A1 =[-34.74 -28.83 203.55];
% F = [-145.29 -15.18 142.66];
%F A2 A3 A4 A5...������ϵO�ı�ʾ A1 A4 A5 һ�� A2 A3һ�� 
% A =[F        ;
%         -34.74 -35.84 138.18;%A4
%         -34.74 -34.81 171.72;
%         -34.74 154.77 190.51;%A2
%        -34.74 162.62 153.22]%A3];%A5
 %B1 B2 B3 B4 B5 ������ϵC�ı�ʾ��
 %�ƶ�ƽ̨�����3����Ϊ B1 B3 B5
cB = [-2 0 -2;
          1 -2 2;
          2 -2 -2;
          1 2 2;
          2 3 -2 ];%2 3 -2
%  cB=15*cB;


Tspan=50;
dalpha = pi/4/Tspan;
% dalpha = 0;
stringL=zeros(Tspan,5);
for k=1:1:Tspan
alpha = alpha +dalpha;
% ���㲿��
cBT=cB';
AT=A';
R = trotz(gamma)*troty(beta)*trotx(alpha);
C1 = homtrans(R,C0);%��ת˳�� Z��Y��X
T0=[ eye(3,3) ,zeros(3,1);zeros(1,3),1];
% T1��ʾ���ĵ�λ�� �� AcT
T1 = transl(C1')*trotz(gamma)*troty(beta)*trotx(alpha);
BT = homtrans(T1,cBT);
%  �������ߵĳ��� A1B1
ML = AT-BT;
for i=1:1:5
    stringL(k,i)=norm(ML(:,i));
end

%  ��ͼ����
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time (s)') 
ylabel('Displacement ') 
trplot(T0,'frame','O','color','r');
axis([-1.2*L 1.2*L -1.2*L 1.2*L  -1.2*L 1.2*L ]);
hold on
trplot(T1,'frame','C','color','b');
% plot3(A1(1),A1(2),A1(3),'o','color','b','MarkerSize',10,'MarkerFaceColor','g');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  �������� Ai ��Bi����
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% ����Bi֮�������
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(3,4)=0;%1 4
LinkB(4,3)=0;
gplot23D(LinkB,BT');
% ����OC����
OC =[[0;0;0] C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
%���� A1F����
A1F=[A1' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end 

stringL
t=1:1:Tspan;
figure(100),
plot(t,stringL,'linewidth',1.1);
axis([1,50,10,24]);
set(gca,'linewidth',1.1,'fontsize',14,'fontname','����')
xlabel('ʱ��/\fontname{Times New Roman}unit');
ylabel('����/\fontname{Times New Roman}unit');
legend('FB1', 'A2B2','A3B3','A4B4','A5B5','fontname','Times');
set (gca,'position',[0.1,0.12,0.7,0.8] );
title('�����߳�����ʱ��仯ͼ');

motorR = 2;
%�Գ�ʼ״̬�ĳ���Ϊ��׼
motorTheta=zeros(Tspan,5);
for i=1:1:Tspan
    motorTheta(i,:)=(stringL(i,:)-stringL(1,:))/2*180/pi;
%     motorTheta(i,:)=(stringL(i,:)-stringL(1,:))/2;
end
% motorTheta
figure(200),
plot(t,motorTheta,'linewidth',1.1);
set(gca,'linewidth',1.1,'fontsize',14,'fontname','����');
title('�����ת����ʱ��仯ͼ');
xlabel('ʱ��/\fontname{Times New Roman}unit');
ylabel('���ת��/\fontname{Times New Roman}��');
legend('A1', 'A2','A3','A4','A5','fontname','Times');
set (gca,'position',[0.1,0.12,0.7,0.8] );
xlswrite('trajactory.xlsx',[motorTheta;stringL]);








